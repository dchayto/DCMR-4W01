/*								kb_input_handler							//
	ros node for handling user input from keyboard; input will be restricted
	to a set of predefined commands (input body twist, adjust gain, potentially
	a set of menu options, etc.

	this module will publish input twist commands to the "input_cmd" topic 
	(to be consumed by the controller node).

	would be nice to be able to handle compound inputs, but this would probably
	require looking for key pressed/key released events, which sounds a lot
	more complicated than i really want to get into here

	NOTE: output from this is %max speed command - consuming node needs to
	multiply the twist vector by the maximum speed to get the actual commanded
	twist (this was done to keep this code agnostic of robot parameters, other
	than those implied by the most basic configuration of the robot)

	"why not use teleop_twist_keyboard?" because i hate myself that's why

	auth: @dchayto
*/

#include <functional>	// for std::bind
#include <termios.h>	// POSIX terminal control definitions 
#include <iostream>
#include <cstring> 		// for strerror
#include <cstdint>		// for uint8_t (redundant include; already in input)
#include <errno.h>		// error msgs
#include <fcntl.h>		// file controls
#include <unistd.h>		// for STDIN_FILENO
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "include/kb_input.hpp"
#include "include/kb_screen_handler.hpp"

#include "robot_params.hpp"	// really don't love having to include this here...

#undef TESTING

class KeyboardHandlerNode : public rclcpp::Node
{
public:
	KeyboardHandlerNode() : Node("kb_handler_node")		{
		// initialize screen & write default gain
		ScreenHandler::initScreen();
		ScreenHandler::writeGain(gain);
		ScreenHandler::hideCursor();
		
		// get starting termios settings
		if (tcgetattr(STDIN_FILENO, &ogTermios) != 0)	{
			RCLCPP_ERROR_STREAM(this->get_logger(),
				"Error " << errno << ": " << std::strerror(errno));
		}
		
		configureRaw(ogTermios);	// PBV; don't want to change ogTermios

		// PUBLISHERS	
		using namespace std::chrono_literals;
		input_twist_publisher 
		    = this->create_publisher<geometry_msgs::msg::Twist>("input_cmd", 1);
		inputTimer = this->create_wall_timer(100ms,
			std::bind(&KeyboardHandlerNode::onKBTimer, this));
	} // </constructor>

	~KeyboardHandlerNode()	{
		RCLCPP_INFO(this->get_logger(), "KeyboardHandlerNode shutting down");
		tcsetattr(STDIN_FILENO, TCSAFLUSH, &ogTermios); // restore termios settings
		ScreenHandler::wipe();
		ScreenHandler::showCursor();
	} // </destructor>

private:
	// member variables
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr input_twist_publisher;
	rclcpp::TimerBase::SharedPtr inputTimer;
	struct termios ogTermios; // termios struct before starting program
	uint8_t gain {50};
	
	// member functions
	void onKBTimer();
	void configureRaw(struct termios rawTerm);
}; // </KeyboardHandlerNode>


void KeyboardHandlerNode::onKBTimer()	{
	using namespace KeyboardConstants;	// kb mappings
	using namespace InputConstants;		// ISR2

	auto kbTwist = geometry_msgs::msg::Twist(); 
	
	// adding hysteresis to first input only to account for keyboard hold-down
	// delay - recommend turning this delay down/off when testing and on robot
	static char ch { ' ' };
	static char prev_ch { ' ' };
	static unsigned int no_command_count { 0 };
	//bool stale_cmd { false };
	uint8_t no_command_threshold { 10 };	// timeout when no commands received

	if (read(STDIN_FILENO, &ch, 1))	{
		no_command_count = 0;
		prev_ch = ch;	
	} else {
		no_command_count++;
	}

	// flush input buffer so old commands don't build up over time
	tcflush(STDIN_FILENO, TCIFLUSH);

	if (ch == prev_ch)	no_command_threshold = 2; // stale command - time out sooner

	// don't want hysteresis on gain controls
	if (ch == GAINUP || ch == GAINUP10 || ch == GAINDN || ch == GAINDN10)
		no_command_threshold = 0;
	
	if (no_command_count > no_command_threshold)	ch = ' '; // default blank 
	
	switch (ch)	{
		// basic movements:
		case FWD:	 	kbTwist.linear.x =  gain*MAX_LINSPEED*0.01;		 break;
		case LFT: 		kbTwist.linear.y =  gain*MAX_LINSPEED*0.01; 	 break;
		case REV: 		kbTwist.linear.x = -gain*MAX_LINSPEED*0.01; 	 break;
		case RHT:		kbTwist.linear.y = -gain*MAX_LINSPEED*0.01;		 break;
		case FWD_RHT: 	kbTwist.linear.x =  gain*ISR2*MAX_LINSPEED*0.01;
						kbTwist.linear.y = -gain*ISR2*MAX_LINSPEED*0.01; break;
		case FWD_LFT:	kbTwist.linear.x =  gain*ISR2*MAX_LINSPEED*0.01; 
						kbTwist.linear.y =  gain*ISR2*MAX_LINSPEED*0.01; break;
		case REV_LFT: 	kbTwist.linear.x = -gain*ISR2*MAX_LINSPEED*0.01; 
						kbTwist.linear.y =  gain*ISR2*MAX_LINSPEED*0.01; break;
		case REV_RHT:	kbTwist.linear.x = -gain*ISR2*MAX_LINSPEED*0.01;
						kbTwist.linear.y = -gain*ISR2*MAX_LINSPEED*0.01; break;
		case TRN_LFT: 	kbTwist.angular.z =  gain*MAX_ANGSPEED*0.01;	 break;
		case TRN_RHT: 	kbTwist.angular.z = -gain*MAX_ANGSPEED*0.01;	 break;
		case GAINUP:	
			gain += 1;
			if (gain > 127) gain = 127;
			ScreenHandler::writeGain(gain);
			break;
		case GAINUP10:	// increase gain by 10
			gain += 10;
			if (gain > 127) gain = 127;		// uint_8, so no chance of overflow
			ScreenHandler::writeGain(gain);
			break;
		case GAINDN:	// decrease gain by 1
			if (gain <= 1) { gain = 0; }
			else { gain -= 1; }
			ScreenHandler::writeGain(gain);
			break;
		case GAINDN10:	// decrease gain by 10
			if (gain <= 10) { gain = 0; }
			else { gain -= 10; }
			ScreenHandler::writeGain(gain);
			break;
		case QT: rclcpp::shutdown(); break; // can't ctrl-c out of this lol
		case STP:	// VEHICLE STOP
		default: break; 	// invalid command; send {0} twist cmd
	}

	// kbTwist.linear.z = 0;  // don't care about value
	// kbTwist.angular.x = 0;  // don't care about value
	// kbTwist.angular.y = 0;  // don't care about value
	
	#ifdef TESTING
		RCLCPP_INFO(this->get_logger(), "Key registered: %c\n\n", ch);
	#endif

	this->input_twist_publisher->publish(kbTwist);	 // publish command
	
} // </onKBTimer>

void KeyboardHandlerNode::configureRaw(struct termios rawtty)	{
	// copied in by value, so not worried about changing
	cfmakeraw(&rawtty);

	rawtty.c_cflag &= ~CSTOPB;	// clear second stop bit

	rawtty.c_iflag &= ~(IXOFF | IXANY);	// turn off s/w flow control
	
	rawtty.c_cc[VTIME] = 0; // turn off any buffering + waiting for input 
	rawtty.c_cc[VMIN]  = 0; // should return '0' on no input

	tcsetattr(STDIN_FILENO, TCSANOW, &rawtty);
} // </configureRaw>

int main(int argc, char** argv)	{
	rclcpp::init(argc, argv);
	auto kbNode = std::make_shared<KeyboardHandlerNode>();
	rclcpp::spin(kbNode);
	rclcpp::shutdown();

	return 0;
}
