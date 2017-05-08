// Monitors execution of a launch file
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef ROSMON_MONITOR_MONITOR_H
#define ROSMON_MONITOR_MONITOR_H

#include "../fd_watcher.h"
#include "../launch/launch_config.h"

#include "node_monitor.h"

#include <boost/signals2.hpp>

#include <ros/node_handle.h>

namespace rosmon
{

namespace monitor
{

class Monitor
{
public:
public:
	explicit Monitor(const launch::LaunchConfig::ConstPtr& config, const FDWatcher::Ptr& watcher);
	~Monitor();

	void setParameters();
	void start();
	void shutdown();
	void forceExit();
	bool allShutdown();

	inline bool ok() const
	{ return m_ok; }

	const std::vector<NodeMonitor::Ptr>& nodes() const
	{ return m_nodes; }
	std::vector<NodeMonitor::Ptr>& nodes()
	{ return m_nodes; }

	boost::signals2::signal<void(std::string,std::string)> logMessageSignal;
private:
	void log(const char* fmt, ...) __attribute__((format (printf, 2, 3)));

	void handleRequiredNodeExit(const std::string& name);

	launch::LaunchConfig::ConstPtr m_config;

	ros::NodeHandle m_nh;
	FDWatcher::Ptr m_fdWatcher;

	std::vector<NodeMonitor::Ptr> m_nodes;

	bool m_ok;
};

}

}

#endif
