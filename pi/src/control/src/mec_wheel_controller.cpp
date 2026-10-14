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
#include <chrono>		// for ms
#include <cmath>		// for std::abs
#include <limits>		// for std::numeric_limits<int8_t>::max()	
#include <algorithm> 	// for std::max

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "interface/msg/wheelspeed.hpp"

#include "include/mec_wheel_controller.hpp" 	// non-member helpers/consts
#include "PID.hpp"	// generic PID controller structure
#include "robot_params.hpp"		// for robot parameters

#undef INP_MESSAGE_TESTING			// enables ROS message writeouts
#define BEL_MESSAGE_TESTING			// enables ROS message writeouts
#define CONTROLLER_IO_TESTING	// enables writeouts of controller I/O

class MecWheelControllerNode : public rclcpp::Node	{
public:
	MecWheelControllerNode() : Node("mec_controller_node")		{
//		std::cout << belTwist.x << " " << belTwist.y << " " << belTwist.w << std::endl;
		// SUBSCRIBERS
		input_twist_subscription = this->create_subscription<geometry_msgs::msg::Twist>
			("input_cmd", 1,
			[this](const geometry_msgs::msg::Twist& icMsg)
			{
				cmdTwist.x = icMsg.linear.x;
				cmdTwist.y = icMsg.linear.y;
				cmdTwist.w = icMsg.angular.z;

				timeout_counter = 0;
				
				#ifdef INP_MESSAGE_TESTING
				RCLCPP_INFO(this->get_logger(), 
					"Received cmd: {vx: %f | vy: %f | w: %f}",
					cmdTwist.x, cmdTwist.y, cmdTwist.w);
				#endif
			});

		measured_twist_subscription = this->create_subscription<nav_msgs::msg::Odometry>
			("/odometry/filtered", 1,
			[this](const nav_msgs::msg::Odometry& btMsg)	{
				// belief twist, from encoder odom + IMU readings (IO estimate)

				belTwist.x = btMsg.twist.twist.linear.x;
				belTwist.y = btMsg.twist.twist.linear.y;
				belTwist.w = btMsg.twist.twist.angular.z;

				if (belTwist.x > MAX_LINSPEED) belTwist.x = MAX_LINSPEED;
				if (belTwist.x < -MAX_LINSPEED) belTwist.x = -MAX_LINSPEED;

				if (belTwist.y > MAX_LINSPEED) belTwist.y = MAX_LINSPEED;
				if (belTwist.y < -MAX_LINSPEED) belTwist.y = -MAX_LINSPEED;

				if (belTwist.w > MAX_ANGSPEED) belTwist.w = MAX_ANGSPEED;
				if (belTwist.w < -MAX_ANGSPEED) belTwist.w = -MAX_ANGSPEED;

				// should consider what happens when sensors break down (or, 
				// stops receiving valid measured twist for other reasons)

				#ifdef BEL_MESSAGE_TESTING
				RCLCPP_INFO(this->get_logger(), 
					"Received bel: {vx: %f | vy: %f | w: %f}",
					belTwist.x, belTwist.y, belTwist.w);
				#endif
			});

		prev = this->get_clock()->now();	// initialize timestep variable

		// PUBLISHERS
		using namespace std::chrono_literals;
		ws_publisher = this->create_publisher<interface::msg::Wheelspeed>("wheelspeed", 1);
		wsTimer = this->create_wall_timer(50ms,
			[this]()	{
				// note: gain order is kp, ki, kd
				// don't love this, consider cleaning up
				static PID vxPID{0.1, 0.0, 0.0, 5e-6};
				static PID vyPID{5e-2, 0.0, 0.0, 5e-3};	// vy likes to slip
				static PID wzPID{0.1, 0.0, 0.0, 5e-6};

				// if command changed, reset PID params	
				static twist prevTwist {};
				if (prevTwist.x != cmdTwist.x)	{ vxPID.reset(); }
				if (prevTwist.y != cmdTwist.y)	{ vyPID.reset(); }
				if (prevTwist.w != cmdTwist.w)	{ wzPID.reset(); }
				prevTwist = cmdTwist;

				// get timestep	
				static rclcpp::Time now; 
				static double dt;
				now = this->get_clock()->now();	
				dt = (now - prev).seconds();
				prev = now;

				// generate error signal then send to PIDs
				ctrlTwist.x = cmdTwist.x + vxPID.correct(cmdTwist.x - belTwist.x, dt);
				ctrlTwist.y = cmdTwist.y + vyPID.correct(cmdTwist.y - belTwist.y, dt);
				ctrlTwist.w = cmdTwist.w + wzPID.correct(cmdTwist.w - belTwist.w, dt);
				
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
				
				// publish wheelspeeds
				auto wsMsg = interface::msg::Wheelspeed();
				wsMsg.front_right 	= frUS;
				wsMsg.front_left 	= flUS;
				wsMsg.back_right 	= brUS;
				wsMsg.back_left 	= blUS;
				this->ws_publisher->publish(wsMsg);

				// reset messages so not running off stale inputs
				if (timeout_counter > input_timeout)	{
					cmdTwist.x = 0; cmdTwist.y = 0; cmdTwist.w = 0;
				}
				timeout_counter++;
				belTwist.x = 0; belTwist.y = 0; belTwist.w = 0;

			});
	} // </constructor>

	~MecWheelControllerNode()	{
		RCLCPP_INFO(this->get_logger(), "MecWheelControllerNode shutting down.");
	} // </destructor>

private:
	// member variables
	twist cmdTwist {0.0, 0.0, 0.0}; 	// commanded twist [x, y, w]
	twist belTwist {0.0, 0.0, 0.0}; 	// current motion belief [x, y, w]
	twist ctrlTwist {0.0, 0.0, 0.0};	// control signal twist [x, y, w]

	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr input_twist_subscription;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr measured_twist_subscription;
	rclcpp::Publisher<interface::msg::Wheelspeed>::SharedPtr ws_publisher;
	rclcpp::TimerBase::SharedPtr wsTimer;
	rclcpp::Time prev;

	uint8_t timeout_counter { 0 };
	const uint8_t input_timeout { 128 };	

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


////////////////////////////// KINEMATIC MODELS //////////////////////////////
// using output from controller, calculate wheelspeeds based on kin model
//
// IGNORING TRANSFORMS TO WORLD FRAME - going to handle any world:body 
// transforms on world-side system (i.e., use robot pose to transform
// desired global position/velocity command to appropriate body frame command
// i.e., considering body frame for this module)
//
// NOTE: considering:
//			- driving direction as x 
//			- vertical up as z (sense for rotation, RHR)
//			- left as y (by right hand naming convention)
//			- theta(world->body) as 0rad (i.e., not accounting for here)
//			- thata(body->wheel) as 0rad (driving direction of wheel frame 
// 			  parallel to driving direction of robot frame)
// 			- body frame to be centered on trackwidth and wheelbase
//	considering these, frame relation matrix becomes:
//		[c(0)		s(0)		x_wheel*s(0) - y_wheel*c(0)]
//		[-s(0)		c(0)		x_wheel*c(0) + y_wheel*s(0)]
//
//		[1			0			-y_wheel]
//		[0			1			 x_wheel]
//  where x_wheel, -y_wheel are the wheel's centre position relative to the
//  body frame of the robot, in (i.e., position in {b})
//
//  overall, v_drive simplfies to	 (1*v_x + 0 + -y_wheel*w_z) 
//								   + MEC_ANGLE * (0 + 1*v_y +  x_wheel*w_z)
//  should really precompute as much of this as possible; everything except
//  v_x, v_y, and w_z are constant by setup
//
//  solved on paper; works out to:
// 		u_drive = (1/r)*v_x + (g)*v_y + (x_wheel*g - y_wheel*g)*w_z
//  where g = tan(MEC_ANGLE) / r
//
//  SHOULD PROBABLY UPDATE THIS CODE TO USE STATIC TF2 PUBLISHER
double MecWheelControllerNode::getFrontRightWS()	{
	// remember to use +MEC_ANGLE, +X_WHEEL, -Y_WHEEL
	static const double G 			{ std::tan(+MEC_ANGLE) / WHEEL_RADIUS };
	static const double X_WHEEL 	{ +WHEELBASE / 2.0 };
	static const double Y_WHEEL 	{ -TRACK_WIDTH / 2.0 };
	static const double G_X 		{ 1.0 / WHEEL_RADIUS };
	// static const double G_Y 		{ G };
	static const double G_W			{ X_WHEEL*G - Y_WHEEL*G };

	return G_X * ctrlTwist.x + G * ctrlTwist.y + G_W * ctrlTwist.w;
}

double MecWheelControllerNode::getFrontLeftWS()		{
	// remember to use -MEC_ANGLE, +X_WHEEL, +Y_WHEEL
	static const double G 			{ std::tan(-MEC_ANGLE) / WHEEL_RADIUS };
	static const double X_WHEEL 	{ +WHEELBASE / 2.0 };
	static const double Y_WHEEL 	{ +TRACK_WIDTH / 2.0 };
	static const double G_X 		{ 1.0 / WHEEL_RADIUS };
	// static const double G_Y 		{ G };
	static const double G_W			{ X_WHEEL*G + Y_WHEEL*G };

	return G_X * ctrlTwist.x + G * ctrlTwist.y + G_W * ctrlTwist.w;
}

double MecWheelControllerNode::getBackRightWS()		{
	// remember to use -MEC_ANGLE, -X_WHEEL, -Y_WHEEL
	static const double G 			{ std::tan(-MEC_ANGLE) / WHEEL_RADIUS };
	static const double X_WHEEL 	{ -WHEELBASE / 2.0 };
	static const double Y_WHEEL 	{ -TRACK_WIDTH / 2.0 };
	static const double G_X 		{ 1.0 / WHEEL_RADIUS };
	// static const double G_Y 		{ G };
	static const double G_W			{ X_WHEEL*G + Y_WHEEL*G };

	return G_X * ctrlTwist.x + G * ctrlTwist.y + G_W * ctrlTwist.w;
}

double MecWheelControllerNode::getBackLeftWS()		{
	// remember to use +MEC_ANGLE, -X_WHEEL, +Y_WHEEL
	static const double G 			{ std::tan(+MEC_ANGLE) / WHEEL_RADIUS };
	static const double X_WHEEL 	{ -WHEELBASE / 2.0 };
	static const double Y_WHEEL 	{ +TRACK_WIDTH / 2.0 };
	static const double G_X 		{ 1.0 / WHEEL_RADIUS };
	// static const double G_Y 		{ G };
	static const double G_W			{ X_WHEEL*G - Y_WHEEL*G };

	return G_X * ctrlTwist.x + G * ctrlTwist.y + G_W * ctrlTwist.w;
}

