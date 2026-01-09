/* 							due_hw_interface.cpp						    //
	ros node for interfacing with arduino due (sending wheel speeds, retrieving
	encoder and other sensor data)

	node will listen on input_cmd topic (from mech_wheel_controller) and feed
	framed message to due through UART interface (USBA-USBB)

	data from due will also be read in through UART and published to various
	topics, as appropriate (e.g., publishing encoder data, sensors, OBD, etc.)

	auth: @dchayto
*/
#undef ROS2 // don't want to deal with this until serial comms worked out

#include <termios.h>	// POSIX terminal control definitions	
#include <unistd.h> 	// write(), read(), close()
#include <fcntl.h>	// file controls 
#include <errno.h>	// error msgs
#include <iostream>
#include <cstdlib>
#include <cstring>	// for strerror

#include <chrono> // for sleeping program during testing - can prob delete after
#include <thread> // ^^

#include "rclcpp/rclcpp.hpp"

#include "UARTmsgs.hpp"

#define TESTING		// outputs additional messages to help with debugging


class DueInterfaceNode : public rclcpp::Node
{
public:
	DueInterfaceNode()
	  : Node("due_interface_node")
	  , ws_ { 0, 0, 0, 0 }
	{
		openPort();

		// SUBSCRIBERS
		ws_subscription = this->create_subscription<DCMR_4W01::msg::Wheelspeed>
			("wheelspeed", 1,
			[this](const DCMR_4W01::msg::Wheelspeed& wsMsg)
			{
				// set ws_ based on received message, send to due
				this->ws_.setWheelSpeed(wsMsg->front_right, wsMsg->front_left, 
							wsMsg->back_right, wsMsg->back_left);
				writeUART(this->ws_.msg, UARTmsgs::WheelSpeed.MSG_SIZE);
				#ifdef TESTING
				RCLCPP_INFO(this->get_logger(),
						"Received wheelspeed: <%PRId8 %PRId8 %PRId8 %PRId8>"
						, wsMsg->front_right, wsMsg->front_left
						, wsMsg->back_right, wsMsg->back_left); 
				#endif
			};)
		
		/* commenting out until actually using encoder odom
		// PUBLISHERS
		odom_publisher = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
		encoderTimer = this->create_wall_timer(1s, 
			[this]()
			{
				// read odom message from due, publish to topic 
				readUART();		
				//... message should be nav_msgs::msg::Odometry
				auto odomMsg = nav_msgs::msg::Odometry();
				odomMsg.header = ;
				odomMsg.child_frame_id = ;
				odomMsg.pose = ;
				odomMsg.twist = ;
				this->odom_publisher->publish(odomMsg);
			};)
		*/
	}
	
	~DueInterfaceNode()
	{
		RCLCPP_INFO(this->get_logger(), "DueInterfaceNode shutting down.");
		closePort();
	}

	static const char UART_PORT[] = "/dev/ttyACM0";

private:
	// member variables
	int serialPort; 
	UARTmsgs::WheelSpeed ws_;
	rclcpp::Subscription<DCMR_4W01::msg::Wheelspeed>::SharedPtr ws_subscription;
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;
	rclcpp::TimerBase::SharedPtr encoderTimer;

	// helper functions
	void openPort();
	void closePort();
	void readUART(char * buf, size_t bufsize);
	void writeUART(char * msg, size_t msgsize);
};

void DueInterfaceNode::closePort()	{
	// move this line into ROS2 node destructor at some point
	// not sure if i have to check for valid file before closing
	close(serialPort);
}

void DueInterfaceNode::openPort()	{
	serialPort = open(UART_PORT, O_RDWR);

	if (serialPort < 0)	{ // should return error code, or set an isValid param
		std::cerr << "Error " << errno << ": " << std::strerror(errno) << std::endl;
	} // tbh, should probably exit function at this point... leaving for now
	// actually should prob just display as a ROS warning

	struct termios tty;
	if (tcgetattr(serialPort, &tty) != 0)	{ // should return error code
		std::cerr << "Error " << errno << ": " << std::strerror(errno) << std::endl;
	}

	// configuring termios; prob don't need a lot of these, but shouldn't hurt
	cfmakeraw(&tty);
	tty.c_cflag &= ~CSTOPB;		// clear second stop bit
	tty.c_cflag &= ~CRTSCTS;	// disable RTS flow control
	tty.c_cflag |= CREAD | CLOCAL; 	// enable READ, ignore control lines

	tty.c_lflag &= ~(ECHOE); 	// disable echoing

	tty.c_iflag &= ~(IXOFF | IXANY);		// turn off s/w flow ctrl
	
	tty.c_oflag &= ~ONLCR;	// prevent conversion of nl to lfo

	tty.c_cc[VTIME] = 0;
	tty.c_cc[VMIN]	= 0; // trusting ROS2 callbacks to handle r/w scheduling

	// set i/o baud rates to 57600; consider increasing if no issues
	cfsetispeed(&tty, B57600);	
	cfsetospeed(&tty, B57600);

	// maybe should change to TCSANOW - went drain because don't want 2 sets
	// of a single type of data making it though on the same message
	if (tcsetattr(serialPort, TCSADRAIN, &tty) != 0)	{
		std::cerr << "Error " << errno << ": " << std::strerror(errno) << std::endl;
	}
}

void DueInterfaceNode::readUART(char * buf, size_t bufsize)	{
	read(serialPort, buf, bufsize);
}

void writeUART(char * msg, size_t msgsize)	{
	write(serialPort, msg, msgsize); 
}

int main(int argc, char** argv)	{
	/* FOR TESTING BASIC MESSAGE PASSING 
	openPort();
	// NOTE: arduino seems to need ~150ms setup time more than pi to init port
	std::this_thread::sleep_for(std::chrono::milliseconds(100)); 
	UARTmsgs::WheelSpeed k { 0, 0, 0, 0 };
	for (int i = -128; i <= 127; ++i)	{
		k.FR = k.FL = k.BR = k.BL = i;
		k.encodeMsg();
		std::cout << "Sending message: " << k.msg << std::endl;
		writeUART(k.msg, UARTmsgs::WheelSpeed::MSG_SIZE);
		std::this_thread::sleep_for(std::chrono::milliseconds(100)); 
		readUART(k.msg, UARTmsgs::WheelSpeed::MSG_SIZE);
		std::cout << "Receiving message: " << k.msg << std::endl << std::endl;
		std::this_thread::sleep_for(std::chrono::milliseconds(100)); 
	}
	closePort();
	*/
	
	rclcpp::init(argc, argv);
	auto dueNode = std::make_shared<DueInterfaceNode>("due_interface_node");
	rclcpp::spin(dueNode);
	rclcpp::shutdown();

	return 0;
} // main
