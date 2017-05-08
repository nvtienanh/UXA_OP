// Monitors a single node process
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "node_monitor.h"

#include <sys/types.h>
#include <sys/wait.h>
#include <sys/resource.h>
#include <sys/utsname.h>
#include <sys/prctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <stdarg.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <pty.h>
#include <wordexp.h>
#include <glob.h>

#include <sstream>

#include <boost/tokenizer.hpp>
#include <boost/range.hpp>
#include <boost/algorithm/string.hpp>

#define TASK_COMM_LEN 16 // from linux/sched.h

static std::runtime_error error(const char* fmt, ...)
{
	va_list args;
	va_start(args, fmt);

	char str[1024];

	vsnprintf(str, sizeof(str), fmt, args);

	va_end(args);

	return std::runtime_error(str);
}

namespace rosmon
{
namespace monitor
{

NodeMonitor::NodeMonitor(const launch::Node::ConstPtr& launchNode, const FDWatcher::Ptr& fdWatcher, ros::NodeHandle& nh)
 : m_launchNode(launchNode)
 , m_fdWatcher(fdWatcher)
 , m_rxBuffer(4096)
 , m_pid(-1)
 , m_exitCode(0)
 , m_command(CMD_STOP) // we start in stopped state
 , m_restarting(false)
{
	m_restartTimer = nh.createWallTimer(ros::WallDuration(1.0), boost::bind(&NodeMonitor::start, this));
	m_stopCheckTimer = nh.createWallTimer(ros::WallDuration(5.0), boost::bind(&NodeMonitor::checkStop, this));
}

NodeMonitor::~NodeMonitor()
{
	if(m_pid != -1)
	{
		kill(m_pid, SIGKILL);
	}
}

std::vector<std::string> NodeMonitor::composeCommand() const
{
	if(m_launchNode->executable().empty())
	{
		throw error("Could not find node '%s' in package '%s'",
			m_launchNode->type().c_str(), m_launchNode->package().c_str()
		);
	}

	// Start with the launch prefix...
	std::vector<std::string> cmd = m_launchNode->launchPrefix();

	// add executable file
	cmd.push_back(m_launchNode->executable());

	// add extra arguments from 'args'
	auto args = m_launchNode->extraArguments();
	std::copy(args.begin(), args.end(), std::back_inserter(cmd));

	// add parameter for node name
	cmd.push_back("__name:=" + m_launchNode->name());

	// and finally add remappings.
	for(auto map : m_launchNode->remappings())
	{
		cmd.push_back(map.first + ":=" + map.second);
	}

	return cmd;
}

void NodeMonitor::start()
{
	m_command = CMD_RUN;

	m_stopCheckTimer.stop();
	m_restartTimer.stop();
	m_restarting = false;

	if(running())
		return;

	ROS_INFO("rosmon: starting '%s'", m_launchNode->name().c_str());

	int pid = forkpty(&m_fd, NULL, NULL, NULL);
	if(pid < 0)
		throw error("Could not fork with forkpty(): %s", strerror(errno));

	if(pid == 0)
	{
		std::vector<std::string> cmd = composeCommand();

		char* path = strdup(cmd[0].c_str());

		std::vector<char*> ptrs(cmd.size());

		for(unsigned int i = 0; i < cmd.size(); ++i)
			ptrs[i] = strdup(cmd[i].data());
		ptrs.push_back(nullptr);

		if(!m_launchNode->namespaceString().empty())
			setenv("ROS_NAMESPACE", m_launchNode->namespaceString().c_str(), 1);

		for(auto& pair : m_launchNode->extraEnvironment())
			setenv(pair.first.c_str(), pair.second.c_str(), 1);

		// Try to enable core dumps
		if(m_launchNode->coredumpsEnabled())
		{
			rlimit limit;
			if(getrlimit(RLIMIT_CORE, &limit) == 0)
			{
				// only modify the limit if coredumps are disabled entirely
				if(limit.rlim_cur == 0)
				{
					limit.rlim_cur = limit.rlim_max;
					setrlimit(RLIMIT_CORE, &limit);
				}
			}
		}
		else
		{
			// Disable coredumps
			rlimit limit;
			if(getrlimit(RLIMIT_CORE, &limit) == 0)
			{
				limit.rlim_cur = 0;
				setrlimit(RLIMIT_CORE, &limit);
			}
		}

		// Allow gdb to attach
		prctl(PR_SET_PTRACER, PR_SET_PTRACER_ANY);

		if(execvp(path, ptrs.data()) != 0)
		{
			std::stringstream ss;
			for(auto part : cmd)
				ss << part << " ";

			log("Could not execute '%s': %s\n", ss.str().c_str(), strerror(errno));
		}
		exit(1);
	}

	// Parent
	m_pid = pid;
	m_fdWatcher->registerFD(m_fd, boost::bind(&NodeMonitor::communicate, this));
}

void NodeMonitor::stop(bool restart)
{
	if(restart)
		m_command = CMD_RESTART;
	else
		m_command = CMD_STOP;

	m_stopCheckTimer.stop();
	m_restartTimer.stop();

	if(!running())
		return;

	// kill(-pid) sends the signal to all processes in the process group
	kill(-m_pid, SIGINT);

	m_stopCheckTimer.start();
}

void NodeMonitor::checkStop()
{
	if(running())
	{
		log("required SIGKILL");
		kill(m_pid, SIGKILL);
	}

	m_stopCheckTimer.stop();
}

void NodeMonitor::restart()
{
	m_stopCheckTimer.stop();
	m_restartTimer.stop();

	if(running())
		stop(true);
	else
		start();
}

void NodeMonitor::shutdown()
{
	if(m_pid != -1)
	{
		kill(-m_pid, SIGINT);
	}
}

void NodeMonitor::forceExit()
{
	if(m_pid != -1)
	{
		kill(-m_pid, SIGKILL);
	}
}

bool NodeMonitor::running() const
{
	return m_pid != -1;
}

NodeMonitor::State NodeMonitor::state() const
{
	if(running())
		return STATE_RUNNING;
	else if(m_restarting)
		return STATE_WAITING;
	else if(m_exitCode == 0)
		return STATE_IDLE;
	else
		return STATE_CRASHED;
}

void NodeMonitor::communicate()
{
	char buf[1024];
	int bytes = read(m_fd, buf, sizeof(buf));

	if(bytes == 0 || (bytes < 0 && errno == EIO))
	{
		int status;

		while(1)
		{
			if(waitpid(m_pid, &status, 0) > 0)
				break;
			else
			{
				if(errno == EINTR || errno == EAGAIN)
					continue;

				throw error("%s: Could not waitpid(): %s", m_launchNode->name().c_str(), strerror(errno));
			}
		}

		if(WIFEXITED(status))
		{
			log("%s exited with status %d", name().c_str(), WEXITSTATUS(status));
			ROS_INFO("rosmon: %s exited with status %d", name().c_str(), WEXITSTATUS(status));
			m_exitCode = WEXITSTATUS(status);
		}
		else if(WIFSIGNALED(status))
		{
			log("%s died from signal %d", name().c_str(), WTERMSIG(status));
			ROS_ERROR("rosmon: %s died from signal %d", name().c_str(), WTERMSIG(status));
			m_exitCode = 255;
		}

#ifdef WCOREDUMP
		if(WCOREDUMP(status))
		{
			// We have a chance to find the core dump...
			log("%s left a core dump", name().c_str());
			gatherCoredump(WTERMSIG(status));
		}
#endif

		m_pid = -1;
		m_fdWatcher->removeFD(m_fd);
		close(m_fd);
		m_fd = -1;

		if(m_command == CMD_RESTART || (m_command == CMD_RUN && m_launchNode->respawn()))
		{
			if(m_command == CMD_RESTART)
				m_restartTimer.setPeriod(ros::WallDuration(1.0));
			else
				m_restartTimer.setPeriod(m_launchNode->respawnDelay());

			m_restartTimer.start();
			m_restarting = true;
		}

		exitedSignal(name());

		return;
	}

	if(bytes < 0)
		throw error("%s: Could not read: %s", name().c_str(), strerror(errno));

	for(int i = 0; i < bytes; ++i)
	{
		m_rxBuffer.push_back(buf[i]);
		if(buf[i] == '\n')
		{
			m_rxBuffer.push_back(0);
			m_rxBuffer.linearize();

			auto one = m_rxBuffer.array_one();
			logMessageSignal(name(), one.first);

			m_rxBuffer.clear();
		}
	}
}

void NodeMonitor::log(const char* fmt, ...)
{
	static char buf[512];

	va_list v;
	va_start(v, fmt);

	vsnprintf(buf, sizeof(buf), fmt, v);

	va_end(v);

	logMessageSignal(name(), buf);
}

static boost::iterator_range<std::string::const_iterator>
corePatternFormatFinder(std::string::const_iterator begin, std::string::const_iterator end)
{
	for(; begin != end && begin+1 != end; ++begin)
	{
		if(*begin == '%')
			return boost::iterator_range<std::string::const_iterator>(begin, begin+2);
	}

	return boost::iterator_range<std::string::const_iterator>(end, end);
}

void NodeMonitor::gatherCoredump(int signal)
{
	char core_pattern[256];
	int core_fd = open("/proc/sys/kernel/core_pattern", O_RDONLY);
	if(core_fd < 0)
	{
		log("could not open /proc/sys/kernel/core_pattern: %s", strerror(errno));
		return;
	}

	int bytes = read(core_fd, core_pattern, sizeof(core_pattern)-1);
	close(core_fd);

	if(bytes < 1)
	{
		log("Could not read /proc/sys/kernel/core_pattern: %s", strerror(errno));
		return;
	}

	if(core_pattern[0] == '|')
	{
		log("You have apport or some other coredump manager installed on your "
		    "system. If you want to use the core dump feature of rosmon, set "
			"/proc/sys/kernel/core_pattern to something like "
			"/tmp/cores/core.%%e.%%p.%%t");
		return;
	}

	core_pattern[bytes-1] = 0; // Strip off the newline at the end

	auto formatter = [&](boost::iterator_range<std::string::const_iterator> match) -> std::string {
		char code = *(match.begin()+1);

		switch(code)
		{
			case '%':
				return "%";
			case 'p':
				return boost::lexical_cast<std::string>(m_pid);
			case 'u':
				return boost::lexical_cast<std::string>(getuid());
			case 'g':
				return boost::lexical_cast<std::string>(getgid());
			case 's':
				return boost::lexical_cast<std::string>(signal);
			case 't':
				return "*"; // No chance
			case 'h':
			{
				utsname uts;
				if(uname(&uts) == 0)
					return uts.nodename;
				else
					return "*";
			}
			case 'e':
				return m_launchNode->type().substr(0, TASK_COMM_LEN-1);
			case 'E':
			{
				std::string executable = m_launchNode->executable();
				boost::replace_all(executable, "/", "!");
				return executable;
			}
			case 'c':
			{
				rlimit limit;
				getrlimit(RLIMIT_CORE, &limit);

				// core limit is set to the maximum above
				return boost::lexical_cast<std::string>(limit.rlim_max);
			}
			default:
				return "*";
		}
	};

	std::string coreGlob = boost::find_format_all_copy(std::string(core_pattern), corePatternFormatFinder, formatter);

	log("Determined pattern '%s'", coreGlob.c_str());

	glob_t results;
	int ret = glob(coreGlob.c_str(), GLOB_NOSORT, 0, &results);

	if(ret != 0 || results.gl_pathc == 0)
	{
		log("Could not find a matching core file :-(");
		globfree(&results);
		return;
	}

	if(results.gl_pathc > 1)
	{
		log("Found multiple matching core files :-(");
		globfree(&results);
		return;
	}

	std::string coreFile = results.gl_pathv[0];
	globfree(&results);

	log("Found core file '%s'", coreFile.c_str());

	std::stringstream ss;

	ss << "gdb " << m_launchNode->executable() << " " << coreFile;

	m_debuggerCommand = ss.str();
}

void NodeMonitor::launchDebugger()
{
	std::string cmd;

	if(coredumpAvailable())
		cmd = m_debuggerCommand;
	else
	{
		std::stringstream ss;
		ss << "gdb -p " << m_pid;
		cmd = ss.str();
	}

	if(getenv("DISPLAY") == 0)
	{
		log("No X11 available, run gdb yourself: %s", cmd.c_str());
	}
	else
	{
		char* envTerm = getenv("ROSMON_DEBUGGER_TERMINAL");
		std::string term = "xterm -e";
		if(envTerm)
			term = envTerm;

		if(system((term + " " + cmd + " &").c_str()) != 0)
		{
			log("Could not launch debugger");
			log("Command: %s", cmd.c_str());
		}
	}
}

}
}
