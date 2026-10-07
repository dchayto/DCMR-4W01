/*								screen_handler.hpp							//
	helper functions to handle screen output for keyboard handler interface

	keeping intentionally simple, and just displaying project name, commands,
	and current gain. everything on screen will be statically written, aside
	from gain, which will be overwritten whenever gain is changed.

	may expand this in the future to allow for display of robot state, when
	more than one divisible state is present (currently just "manual driving"
	state, but there may be some utility in adding 

	this may also be expanded/rewritten in the future to display status
	messages (critical errors, etc.), but currently i'm thinking to have that
	info be in a separate terminal window (i.e., opening a window to display
	all RCLCPP info/warning messages and tiling with other windows)
	  -> LOOK INTO TMUX FOR TILING - see if this will work over ssh (should)
	** NEED TO FIGURE OUT HOW TO FIX TERMINAL WIDTH ON OPEN - don't want fmt
	   to get messed up with resizing windows
*/
#ifndef __KB_SCREEN_HANDLER__
#define __KB_SCREEN_HANDLER__

#include <iostream>
#include <iomanip>
#include <cstdlib> 	// for std::system("clear")
#include <string>
#include <algorithm>

#include "kb_input.hpp"

namespace ScreenHandler	{
	void wipe()	{
		std::system("clear");
	}

	void hideCursor()	{
		std::cout << "\033[?25l" << std::flush;
	}
	
	void showCursor()	{
		std::cout << "\033[?25h" << std::flush;
	}

	void writeGain(uint8_t gain)	{
		// NOTE: this function assumes gain is last value written to screen,
		// and cursor is on final digit of gain value (expressing as 3 chars)
		std::cout << "\b\b\b"; 	// move cursor back 3 chars
		std::cout << std::setw(3) << std::setfill('0') << +gain;
		std::cout << std::flush;
	}

	void initScreen()	{
		wipe();
		// ALSO HANDLE SCREEN FORMATTING - look into this
	
		// NOTE: not supporting custom input schemes. could mess up following
		// code by trying to map a placeholder char somewhere else in the string	
		// if i really cared, could split up into a bunch of different strings
		// and only replace a few chars at a time
		
		// keeping to 64 char width
		std::string scr { 	
"\n       ____  ________  _______        __ ___       ______ ___    " // write string for project title
"\n      / __ ?/ ____/  |/  / __ ?      / // / |     / / __ <  /    "
"\n     / / / / /   / /|_/ / /_/ /_____/ // /| | /| / / / / / /     " 
"\n    / /_/ / /___/ /  / / _, _/_____/__  __/ |/ |/ / /_/ / /      " 
"\n   /_____/?____/_/  /_/_/ |_|        /_/  |__/|__/?____/_/       " 
"\n                                                                 " // end of project title string
"\n       ^                                        QUIT: ~          " // write string for instructions
"\n       w                                                         "
"\n     q   e        <---      --->                                 "
"\n  < a  x  d >       , ?    / .                                   "
"\n     z   c          _/      _               { [0  127] }         "
"\n       s                                   -- - GAIN + ++        "
"\n       v                                        /   " // shorter to allow gain on same line
		};
		
		using namespace KeyboardConstants;
		constexpr char BSLASH_PH 	{'?'}; // placeholder for '\'
		std::replace(scr.begin(), scr.end(), BSLASH_PH, '\\');
		std::replace(scr.begin(), scr.end(), 'w', FWD);
		std::replace(scr.begin(), scr.end(), 'q', FWD_LFT);
		std::replace(scr.begin(), scr.end(), 'e', FWD_RHT);
		std::replace(scr.begin(), scr.end(), 'a', LFT);
		std::replace(scr.begin(), scr.end(), 's', STP);
		std::replace(scr.begin(), scr.end(), 'd', RHT);
		std::replace(scr.begin(), scr.end(), 'z', REV_LFT);
		std::replace(scr.begin(), scr.end(), 'c', REV_RHT);
		std::replace(scr.begin(), scr.end(), 'x', REV);
		
		std::replace(scr.begin(), scr.end(), ',', TRN_LFT);
		std::replace(scr.begin(), scr.end(), '.', TRN_RHT);
	
		std::replace(scr.begin(), scr.end(), ']', GAINUP);
		std::replace(scr.begin(), scr.end(), '}', GAINUP10);
		std::replace(scr.begin(), scr.end(), '[', GAINDN);
		std::replace(scr.begin(), scr.end(), '{', GAINDN10);

		hideCursor();
		std::cout << scr << std::flush;	// send to screen 
	} // </initScreen>
} // </ScreenHandler> 

#endif
