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
#include <fcntl.h>		// file controls 
#include <errno.h>		// error msgs
#include <iostream>
#include <cstdlib>

#include <chrono> // for sleeping program during testing - can prob delete after
#include <thread> // ^^

#ifdef ROS2
#include "rclcpp/rclcpp.hpp"
#endif

//include "UARTmsgs.hpp"

const char UART_PORT[]= "/dev/ttyACM0";
static int serialPort; // move to private member var once node implemented

// obvs move buffers to private member variables once ros node implemented
//char encBuf[256]; 	// buffer for encoder data - decide what a reasonable value is

void closePort()	{
	// move this line into ROS2 node destructor at some point
	// not sure if i have to check for valid file before closing
	close(serialPort);
}

void openPort()	{
	// move this code into ROS2 node contructor at some point
	serialPort = open(UART_PORT, O_RDWR);

	// apparently might run into issues if GeTTY is alreay trying to manage
	// the device - if having troubles communicating, look into disabling
	// (maybe do on pi at this point - don't want to be messing with sys files)

	if (serialPort < 0)	{ // should return error code, or set an isValid param
		std::cerr << "Error " << errno << ": " << strerror(errno) << std::endl;
	} // tbh, should probably exit function at this point... leaving for now
	// actually should prob just display as a ROS warning

	struct termios tty;
	if (tcgetattr(serialPort, &tty) != 0)	{ // should return error code
		std::cerr << "Error " << errno << ": " << strerror(errno) << std::endl;
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

	// set i/o baud rates to 9600; consider increasing if no issues
	cfsetispeed(&tty, B9600);	
	cfsetospeed(&tty, B9600);

	// maybe should change to TCSANOW - went drain because don't want 2 sets
	// of a single type of data making it though on the same message
	if (tcsetattr(serialPort, TSCADRAIN, &tty) != 0)	{
		std::cerr << "Error " << errno << ": " << strerror(errno) << std::endl;
	}
}

void readUART(char * buf, size_t bufsize)	{
	read(serialPort, buf, bufsize);
}

void writeUART(char * msg, size_t msgsize)	{
	// v probably need to send/recieve these messages as delimited strings
	write(serialPort, msg, msgsize);
}

int main(int argc, char** argv)	{
	openPort();

	// NOTE: arduino seems to need ~150ms setup time more than pi to init port
	std::this_thread::sleep_for(std::chrono::milliseconds(150)); 
	UARTmsgs::WheelSpeed k { 0, 0, 0, 0 };
	UARTmsgs::WheelSpeed j { 0, 0, 0, 0 };
	for (int i = -128; i <= 127; ++i)	{
		k.FR = k.FL = k.BR = k.BL = i;
		k.encodeMsg();
		std::cout << "Sending message: " << k.msg << std::endl;
		writeUART(k.msg, UARTmsgs::MSG_SIZE);
		std::this_thread::sleep_for(std::chrono::milliseconds(500)); 
		readUART(j.msg, UARTmsgs::MSG_SIZE)
		std::cout << "Receiving message: " << j.msg << std::endl;
		std::this_thread::sleep_for(std::chrono::milliseconds(500)); 
	}

	closePort();
	return 0;
} // main
