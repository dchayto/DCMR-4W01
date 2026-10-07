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
#include <chrono>	// for ms

#include "rclcpp/rclcpp.hpp"
#include "control/msg/wheelspeed.hpp"
#include "control/msg/wheeltravel.hpp"

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
			// set ws_rad_s_ based on received message, send to due
			ws_rad_s_.fr_rad_s = wsMsg.front_right;
			ws_rad_s_.fl_rad_s = wsMsg.front_left;
			ws_rad_s_.br_rad_s = wsMsg.back_right;
			ws_rad_s_.bl_rad_s = wsMsg.back_left;

			// FOR SOME REASON, MESSAGES AREN'T SERIALIZING PROPERLY
			// first message will write fine, but second message doesn't write
			// over first message... need to investigate
			static size_t write_size;
			write_size = serialMSG::serializePacket(0, 
				write_buffer, ws_rad_s_);
			writeSerial(write_buffer, write_size);

			/////////////////////  TESTING MESSAGES //////////////////////
			#ifdef SUBSCRIPTION_RECEIVE_TESTING
			std::cout << "FRONT RIGHT:\t" << wsMsg.front_right << "rad/s" << std::endl;
			std::cout << "FRONT LEFT:\t" << wsMsg.front_left << "rad/s" << std::endl;
			std::cout << "BACK RIGHT:\t" << wsMsg.back_right << "rad/s" << std::endl;
			std::cout << "BACK LEFT:\t" << wsMsg.back_left << "rad/s" << std::endl;
			std::cout << std::flush;
			#endif
			#ifdef MC_MESSAGE_TESTING
			std::cout << "Attempting to write wheelspeed message (" 
					<< write_size << "b):" << std::endl;
			std::cout << "FRONT RIGHT:\t" << ws_rad_s_.fr_rad_s << "rad/s" << std::endl;
			std::cout << "FRONT LEFT:\t" << ws_rad_s_.fl_rad_s << "rad/s" << std::endl;
			std::cout << "BACK RIGHT:\t" << ws_rad_s_.br_rad_s << "rad/s" << std::endl;
			std::cout << "BACK LEFT:\t" << ws_rad_s_.bl_rad_s << "rad/s" << std::endl;
			//std::cout.write(static_cast<int>(write_buffer), write_size);
			for (size_t i = 0; i < write_size; ++i)	std::cout 
				<< static_cast<int>(write_buffer[i]) << " ";
			std::cout << std::endl << std::flush;
			#endif
			//////////////////////////////////////////////////////////////
		});
		
		// PUBLISHERS
		using namespace std::chrono_literals;
		wt_publisher = this->create_publisher<control::msg::Wheeltravel>
					("wheeltravel", 10); 
					encoderTimer = this->create_wall_timer(50ms, 
		[this]()	{
			
			static constexpr uint8_t TEMPBUF_SIZE { 64 };
			static uint8_t serial_buffer[TEMPBUF_SIZE];
			
			static constexpr uint8_t ENCBUF_SIZE { 128 };
			static uint8_t enc_buffer[ENCBUF_SIZE];
			static uint8_t head { 0 };	// last parsed byte
			static uint8_t tail { 0 };	// last read byte

			static uint8_t cseq { 0 };					// current seqID
			static uint8_t lseq { ENCBUF_SIZE-1 };	// last seqID processed

			// read ardunio serial port data into temp buffer
			static ssize_t bytes_read { 0 };
			bytes_read = readSerial(serial_buffer, TEMPBUF_SIZE);

			// read from temp buffer into ring buffer
			for (ssize_t i = 0; i < bytes_read; ++i)	{
				++tail &= (ENCBUF_SIZE-1);	// need to inc tail first
				enc_buffer[tail] = serial_buffer[i];
			}

			// default wheeltravel to zeros
			wt_rad_.dt = 0; wt_rad_.fr_rad = 0.0; wt_rad_.fl_rad = 0.0;
			wt_rad_.br_rad = 0.0; wt_rad_.bl_rad = 0.0;
			// parse ring buffer, looking for message w/ higher sequence
			for (uint8_t idx = head; idx != tail; ++idx &= (ENCBUF_SIZE-1))	{

				// there's probably a more efficient way of doing this...
				// but can't be at wrapover when going to serialMSG
				for (size_t i = 0; i < serialMSG::WheelTravel::MSG_SIZE; ++i)	{
					serial_buffer[i] = enc_buffer[(idx + i) & (ENCBUF_SIZE - 1)];
				}

				static uint8_t* msgStart { nullptr };
				msgStart = serialMSG::parsePacket(cseq, enc_buffer + idx);

				static uint8_t seqdiff;
				seqdiff = cseq - lseq;

				if (msgStart != nullptr && seqdiff < 0x80)	{
					// valid, not-yet-processed message found; copy into temp buf

					wt_rad_.deserialize(msgStart);
					head = idx;		// if found msg, break loop & process
					break;		// get out of loop so message can be processed
				}
			}
			

			// publish to topic 
			auto wtMsg = control::msg::Wheeltravel();
			wtMsg.dt = wt_rad_.dt;
			wtMsg.front_right = wt_rad_.fr_rad;
			wtMsg.front_left = wt_rad_.fl_rad;
			wtMsg.back_right = wt_rad_.br_rad;
			wtMsg.back_left = wt_rad_.bl_rad;
			this->wt_publisher->publish(wtMsg);
		});
	} // constructor
	
	~DueInterfaceNode()		{
		RCLCPP_INFO(this->get_logger(), "DueInterfaceNode shutting down.");
		closePort();
	} // destructor

private:
	// member variables
	inline static constexpr char SERIAL_PORT[] = "/dev/ttyACM0";
	int serialPort; 
	serialMSG::WheelSpeed ws_rad_s_;
	serialMSG::WheelTravel wt_rad_;
	rclcpp::Subscription<control::msg::Wheelspeed>::SharedPtr ws_subscription;
	rclcpp::Publisher<control::msg::Wheeltravel>::SharedPtr wt_publisher;
	rclcpp::TimerBase::SharedPtr encoderTimer;

	// helper functions
	void openPort();
	void closePort();
	ssize_t readSerial(uint8_t* buf, size_t bufsize);
	ssize_t writeSerial(uint8_t* msg, size_t msgsize);
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

ssize_t DueInterfaceNode::readSerial(uint8_t* buf, size_t bufsize)	{
	return read(serialPort, reinterpret_cast<char*>(buf), bufsize);
}

ssize_t DueInterfaceNode::writeSerial(uint8_t* msg, size_t msgsize)	{
	return write(serialPort, reinterpret_cast<const char*>(msg), msgsize); 
}

int main(int argc, char** argv)	{
	rclcpp::init(argc, argv);
	auto dueNode = std::make_shared<DueInterfaceNode>();
	rclcpp::spin(dueNode);
	rclcpp::shutdown();

	return 0;
} // main
