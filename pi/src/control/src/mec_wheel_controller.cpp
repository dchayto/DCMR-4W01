/*							mec_wheel_controller.cpp				     	//
	ros node for robot controller 

	node will take in a body twist vector representing an input command, as 
	well as a secondary twist vector representing the current actual robot 
	twist, as determined from onboard sensors and wheel odometry

	robot will then convert these values into wheelspeed commands, and send
	to due's hardware interface node 

	note: i feel like a lot of these calculations could/should be using
	floats, but ROS messages seem to use double (f64), so keeping code matching
	to prevent accidental type conversions

	auth: @dchayto
*/

#include <cstdint>
#include <chrono>
#include <cmath>		// for std::abs
#include <limits>		// for std::numeric_limits<int8_t>::max()	
#include <algorithm> 	// for std::max

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "control/msg/wheelspeed.hpp"

#include "include/mec_wheel_controller.hpp" 	// non-member helpers/consts
#include "PID.hpp"	// generic PID controller structure
#include "robot_params.hpp"		// for robot parameters

#undef MESSAGE_TESTING			// enables ROS message writeouts
#define CONTROLLER_IO_TESTING	// enables writeouts of controller I/O

class MecWheelControllerNode : public rclcpp::Node	{
public:
	MecWheelControllerNode() : Node("mech_controller_node")		{
		std::cout << belTwist.x << " " << belTwist.y << " " << belTwist.w << std::endl;
		// SUBSCRIBERS
		input_twist_subscription = this->create_subscription<geometry_msgs::msg::Twist>
			("input_cmd", 1,
			[this](const geometry_msgs::msg::Twist& icMsg)
			{
				cmdTwist.x = icMsg.linear.x;
				cmdTwist.y = icMsg.linear.y;
				cmdTwist.w = icMsg.angular.z;
				
				#ifdef MESSAGE_TESTING
				RCLCPP_INFO(this->get_logger(), 
					"Received cmd: {vx: %f | vy: %f | w: %f}",
					cmdTwist.x, cmdTwist.y, cmdTwist.w);
				#endif
			});
		measured_twist_subscription = this->create_subscription<geometry_msgs::msg::Twist>
			("bel_twist", 1,
			[this](const geometry_msgs::msg::Twist& btMsg)	{
				belTwist.x = btMsg.linear.x;
				belTwist.y = btMsg.linear.y;
				belTwist.w = btMsg.angular.z;

				#ifdef MESSAGE_TESTING
				RCLCPP_INFO(this->get_logger(), 
					"Received bel: {vx: %f | vy: %f | w: %f}",
					belTwist.x, belTwist.y, belTwist.w);
				#endif
			});

		prev = this->get_clock()->now();	// initialize timestep variable

		// PUBLISHERS
		using namespace std::chrono_literals;
		ws_publisher = this->create_publisher<control::msg::Wheelspeed>("wheelspeed", 1);
		wsTimer = this->create_wall_timer(50ms,
			[this]()	{
				// if command changed, reset PID params	
				static twist prevTwist {};
				if (prevTwist.x == cmdTwist.x)	{ vxPID.reset(); }
				if (prevTwist.y == cmdTwist.y)	{ vyPID.reset(); }
				if (prevTwist.w == cmdTwist.w)	{ wzPID.reset(); }
				prevTwist = cmdTwist;

				// get timestep	
				static rclcpp::Time now; 
				static double dt;
				now = this->get_clock()->now();	
				dt = (now - prev).seconds();
				prev = now;

				// generate error signal then send to PIDs
				ctrlTwist.x = vxPID.correct(cmdTwist.x - belTwist.x, dt);
				ctrlTwist.y = vyPID.correct(cmdTwist.y - belTwist.y, dt);
				ctrlTwist.w = wzPID.correct(cmdTwist.w - belTwist.w, dt);
				
				#ifdef CONTROLLER_IO_TESTING
				RCLCPP_INFO(this->get_logger(), 
					"Controller output: {vx: %f | vy: %f | w: %f}",
					ctrlTwist.x, ctrlTwist.y, ctrlTwist.w);
				#endif

				// using controller output, get wheelspeeds (rad/s)
				static double frUS, flUS, brUS, blUS; 
				frUS = getFrontRightWS();
				flUS = getFrontLeftWS();
				brUS = getBackRightWS();
				blUS = getBackLeftWS();

				// scale wheelspeeds if command exceeding max speed 
				// surely there's a better way to do this... oh well!
				static double max_cmd {};
				max_cmd = std::max({std::abs(frUS), std::abs(flUS),
							std::abs(brUS), std::abs(blUS)});
				if (max_cmd > MAX_WHEELSPEED)
				{
					static double wheelSF { 1.0 };
					wheelSF = MAX_WHEELSPEED / max_cmd;
					frUS *= wheelSF;
					flUS *= wheelSF;
					brUS *= wheelSF;
					blUS *= wheelSF;
				}	
				
				// publish wheelspeeds
				auto wsMsg = control::msg::Wheelspeed();
				wsMsg.front_right 	= frUS;
				wsMsg.front_left 	= flUS;
				wsMsg.back_right 	= brUS;
				wsMsg.back_left 	= blUS;
				this->ws_publisher->publish(wsMsg);
			});
	} // </constructor>

	~MecWheelControllerNode()	{
		RCLCPP_INFO(this->get_logger(), "MecWheelControllerNode shutting down.");
	} // </destructor>

private:
	// member variables
	// note: might not be practicable to use velocities directly; should
	// consider just using relative values for body twists, and converting
	// sensor twist value into relative values
	twist cmdTwist {0.0, 0.0, 0.0}; // commanded twist [x, y, w]
	twist belTwist {0.0, 0.0, 0.0}; // belief twist [x, y, w]
	twist ctrlTwist {0.0, 0.0, 0.0};	// control signal twist [x, y, w]

	// note: gain order is kp, ki, kd
	// don't love this, consider cleaning up
	PID vxPID{0.2, 0.0, 0.0};
	PID vyPID{0.2, 0.0, 0.0};
	PID wzPID{0.2, 0.0, 0.0};

	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr input_twist_subscription;
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr measured_twist_subscription;
	rclcpp::Publisher<control::msg::Wheelspeed>::SharedPtr ws_publisher;
	rclcpp::TimerBase::SharedPtr wsTimer;
	rclcpp::Time prev;

	// helper functions
	double getFrontRightWS();
	double getFrontLeftWS();
	double getBackRightWS();
	double getBackLeftWS();
}; // class

int main(int argc, char** argv)	{
	rclcpp::init(argc, argv);
	auto controllerNode = std::make_shared<MecWheelControllerNode>();
	rclcpp::spin(controllerNode);
	rclcpp::shutdown();

	return 0;
} // main
