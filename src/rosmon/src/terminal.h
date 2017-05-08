// Encapsulates terminal control (colors, cursor, ...)
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef TERMINAL_H
#define TERMINAL_H

#include <stdint.h>
#include <string>

namespace rosmon
{

/**
 * @brief Encapsulates terminal control
 *
 * This class enables low-level manipulation of the terminal. It uses the
 * ncurses/terminfo library internally to stay somewhat portable.
 **/
class Terminal
{
public:
	/**
	 * @brief Simple colors
	 *
	 * These colors can be used with the setSimpleForeground(),
	 * setSimpleBackground(), setSimplePair() methods.
	 **/
	enum SimpleColor
	{
		Black,
		Red,
		Green,
		Yellow,
		Blue,
		Magenta,
		Cyan,
		White,
		IntenseBlack,
		IntenseRed,
		IntenseGreen,
		IntenseYellow,
		IntenseBlue,
		IntenseMagenta,
		IntenseCyan,
		IntenseWhite,

		//! 24-step grayscale starts here
		Grayscale = 0xe8
	};

	/**
	 * @brief Terminal escape sequence parser
	 *
	 * This class allows the user to parse Linux escape sequences
	 * (restricted to simple color sequences for now).
	 **/
	class Parser
	{
	public:
		Parser();

		//! parse single character c
		void parse(char c);

		//! parse string
		void parse(const std::string& str);

		//! Apply the current internal state (colors) on the terminal
		void apply(Terminal* term);
	private:
		void parseSetAttributes(const std::string& attrs);

		enum State
		{
			STATE_ESCAPE,
			STATE_TYPE,
			STATE_CSI
		};

		State m_state;
		std::string m_buf;

		int m_fgColor;
		int m_bgColor;
	};

	Terminal();
	~Terminal();

	/**
	 * @brief Set 24-bit foreground color
	 *
	 * This automatically falls back to 256 colors if the terminal does not
	 * support true color.
	 **/
	void setForegroundColor(uint32_t color);

	/**
	 * @brief Set 24-bit background color
	 *
	 * This automatically falls back to 256 colors if the terminal does not
	 * support true color.
	 **/
	void setBackgroundColor(uint32_t color);

	//! hide cursor
	void setCursorInvisible();

	//! restore cursor
	void setCursorVisible();

	/**
	 * Enable/disable automatic echo of keypresses
	 **/
	void setEcho(bool on);

	//! @name Set indexed foreground/background color
	//@{
	void setSimpleBackground(SimpleColor color);
	void setSimpleForeground(SimpleColor color);
	void setSimplePair(SimpleColor fg, SimpleColor bg);
	//@}

	//! Reset fg + bg to standard terminal colors
	void setStandardColors();

	//! Clear characters from cursor to end of line
	void clearToEndOfLine();

	//! Move cursor up by numLines
	void moveCursorUp(int numLines);

	//! Move cursor to start of the line
	void moveCursorToStartOfLine();

	/**
	 * @brief Get current window size
	 *
	 * @return true on success
	 **/
	bool getSize(int* columns, int* rows);

	/**
	 * Returns whether the terminal supports 256 colors. If it does not suppport
	 * 256 colors, you should not use the setForegroundColor() /
	 * setBackgroundColor() functions.
	 **/
	bool has256Colors() const;

	/**
	 * Terminal supports escape codes
	 **/
	bool interactive() const
	{ return m_valid; }

private:
	bool m_valid;
	bool m_256colors;
	bool m_truecolor;

	std::string m_bgColorStr;
	std::string m_fgColorStr;
	std::string m_opStr;
	std::string m_elStr;
	std::string m_upStr;
};

}

#endif
