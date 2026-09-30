/* 							due_hw_interface.cpp						    //
	ros node for interfacing with arduino due (sending wheel speeds, retrieving
	encoder and other sensor data)

	node will listen on input_cmd topic (from mech_wheel_controller) and feed
	framed message to due through serial interface (USBA-USBB)

	data from due will also be read in through serial and published to various
	topics, as appropriate (e.g., publishing encoder data, sensors, OBD, etc.)

	auth: @dchayto
*/
#include <termios.h>	// POSIX terminal control definitions	
#include <unistd.h> 	// write(), read(), close()
#include <fcntl.h>	// file controls 
#include <errno.h>	// error msgs
#include <iostream>
#include <cstdlib>
#include <cstring>	// for strerror

#include "rclcpp/rclcpp.hpp"
#include "control/msg/wheelspeed.hpp"

#include "serialMSG.hpp"	// from common_include folder

// outputs additional messages to help with debugging
#define SUBSCRIPTION_RECEIVE_TESTING
#define MC_MESSAGE_TESTING

class DueInterfaceNode : public rclcpp::Node	
{
public:
	DueInterfaceNode() : Node("due_interface_node")		{
		openPort();

		// SUBSCRIBERS
		ws_subscription = this->create_subscription<control::msg::Wheelspeed>
				("wheelspeed", 1,
				[this](const control::msg::Wheelspeed& wsMsg)		{
			static uint8_t write_buffer[64]; // assuming messages always well under 64b
			// set ws_mrad_s_ based on received message, send to due
			this->ws_mrad_s_.fr_mrad_s = wsMsg.front_right;
			this->ws_mrad_s_.fl_mrad_s = wsMsg.front_left;
			this->ws_mrad_s_.br_mrad_s = wsMsg.back_right;
			this->ws_mrad_s_.bl_mrad_s = wsMsg.back_left;

			static size_t write_size = serialMSG::serializePacket(0, 
				write_buffer, ws_mrad_s_);
			writeSerial(write_buffer, write_size);

			/////////////////////  TESTING MESSAGES //////////////////////
			#ifdef SUBSCRIPTION_RECEIVE_TESTING
			std::cout << "FRONT RIGHT:\t" << wsMsg.front_right << "mrad/s" << std::endl;
			std::cout << "FRONT LEFT:\t" << wsMsg.front_left << "mrad/s" << std::endl;
			std::cout << "BACK RIGHT:\t" << wsMsg.back_right << "mrad/s" << std::endl;
			std::cout << "BACK LEFT:\t" << wsMsg.back_left << "mrad/s" << std::endl;
			std::cout << std::flush;
			#endif
			#ifdef MC_MESSAGE_TESTING
			std::cout << "Attempting to write wheelspeed message (" 
					<< write_size << "b):" << std::endl;
			std::cout.write(reinterpret_cast<const char*>(write_buffer), write_size);
			std::cout << std::endl << std::flush;
			#endif
			//////////////////////////////////////////////////////////////
		});
		
		/* commenting out until actually using encoder odom
		// PUBLISHERS
		odom_publisher = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
		encoderTimer = this->create_wall_timer(1s, 
			[this]()
			{
				// TBH odom should be its own node - this should just publish
				// encoder states

				// read odom message from due, publish to topic 
				readPort();		
				auto odomMsg = nav_msgs::msg::Odometry();
				odomMsg.header = ;
				odomMsg.child_frame_id = ;
				odomMsg.pose = ;
				odomMsg.twist = ;
				this->odom_publisher->publish(odomMsg);
			});
		*/
	} // constructor
	
	~DueInterfaceNode()		{
		RCLCPP_INFO(this->get_logger(), "DueInterfaceNode shutting down.");
		closePort();
	} // destructor

private:
	// member variables
	inline static constexpr char SERIAL_PORT[] = "/dev/ttyACM0";
	int serialPort; 
	serialMSG::WheelSpeed ws_mrad_s_;
	serialMSG::WheelTravel wt_theta_;
	rclcpp::Subscription<control::msg::Wheelspeed>::SharedPtr ws_subscription;
	rclcpp::TimerBase::SharedPtr encoderTimer;

	// helper functions
	void openPort();
	void closePort();
	void readSerial(uint8_t* buf, size_t bufsize);
	void writeSerial(uint8_t* msg, size_t msgsize);
};

void DueInterfaceNode::closePort()	{
	close(serialPort);
}

void DueInterfaceNode::openPort()	{
	serialPort = open(SERIAL_PORT, O_RDWR);

	if (serialPort < 0)	{ // should return error code, or set an isValid param
		RCLCPP_ERROR_STREAM(this->get_logger(), 
			"Error " << errno << ": " << std::strerror(errno));
		// should (bare minimum) add an indicator light or something
	} 

	struct termios tty;
	if (tcgetattr(serialPort, &tty) != 0)	{ // should return error code
		RCLCPP_ERROR_STREAM(this->get_logger(),
			"Error " << errno << ": " << std::strerror(errno));
		// should add an indicator light or something 
	}

	// configuring termios; prob don't need a lot of these, but shouldn't hurt
	cfmakeraw(&tty);
	tty.c_cflag &= ~CSTOPB;		// clear second stop bit

	tty.c_iflag &= ~(IXOFF | IXANY);		// turn off s/w flow ctrl
	
	tty.c_oflag &= ~ONLCR;	// prevent conversion of nl to lfo

	// maybe these should be VTIME 1, VMIN -> sizeof(msg)
	tty.c_cc[VTIME] = 0;
	tty.c_cc[VMIN]	= 0; // trusting ROS2 callbacks to handle r/w scheduling

	// set i/o baud rates to 57600; consider increasing if need more b/w
	cfsetispeed(&tty, B57600);	
	cfsetospeed(&tty, B57600);

	// going with TCSADRAIN since changing baud rate
	if (tcsetattr(serialPort, TCSADRAIN, &tty) != 0)	{
		RCLCPP_ERROR_STREAM(this->get_logger(),
			"Error " << errno << ": " << std::strerror(errno)); 
		// should set indicator light
	}
}

void DueInterfaceNode::readSerial(uint8_t* buf, size_t bufsize)	{
	read(serialPort, reinterpret_cast<char*>(buf), bufsize);
}

void DueInterfaceNode::writeSerial(uint8_t* msg, size_t msgsize)	{
	write(serialPort, reinterpret_cast<const char*>(msg), msgsize); 
}

int main(int argc, char** argv)	{
	rclcpp::init(argc, argv);
	auto dueNode = std::make_shared<DueInterfaceNode>();
	rclcpp::spin(dueNode);
	rclcpp::shutdown();

	return 0;
} // main
