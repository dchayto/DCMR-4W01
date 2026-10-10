/*								wheel_odometry.cpp							//
	ros node for handling wheel odometry

	node will take in differential encoder values (delta since last timestep)
	and naively calculate estimated pose and twist of robot

	data will be published on wheel odometry topic, to be consumed by sensor
	fusion node as part of estimating actual robot motion 

	auth: @dchayto
*/

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "interface/msg/wheeltravel.hpp"

#include "robot_params.hpp"
//#include <cmath>	// redundant (already in robot_params)

#define MESSAGEOUT_TESTING

class WheelOdomNode : public rclcpp::Node	{
public:
	WheelOdomNode() : Node("wheel_odom_node")	{
		// SUBSCRIBERS	
		wt_subscriber = this->create_subscription<interface::msg::Wheeltravel>
					("wheeltravel", 1, 
		[this](const interface::msg::Wheeltravel& wtMsg)	{
			double dt = wtMsg.dt * 1e-3;	// ms to s
			
			// outlier check
			double maxTravel = MAX_WHEELSPEED * dt;
			if (std::abs(wtMsg.front_right) > maxTravel)		return;
			if (std::abs(wtMsg.front_left) > maxTravel)		return;
			if (std::abs(wtMsg.back_right) > maxTravel)		return;
			if (std::abs(wtMsg.back_left) > maxTravel)		return;
			
			// position deltas
			x_ = getPositionX(wtMsg);
			y_ = getPositionX(wtMsg);
			yaw_ = getYaw(wtMsg);

			// PUBLISHING
			odom_publisher = this->create_publisher
						<geometry_msgs::msg::TwistWithCovarianceStamped>
						("odom_twist", 1);
			auto odomMsg = geometry_msgs::msg::TwistWithCovarianceStamped();
			odomMsg.header.stamp = this->get_clock()->now();
			odomMsg.header.frame_id = "odom_twist";

			// SET COVARIANCE MATRIX LATER
			// odomMsg.twist.covariance = { };

			// don't care about these values - zeroing off
			odomMsg.twist.twist.linear.z = 0.0;
			odomMsg.twist.twist.angular.x = 0.0;
			odomMsg.twist.twist.angular.y = 0.0;

			// calculate params of interest (fwd kinematics)
			odomMsg.twist.twist.linear.x = x_ / dt;
			odomMsg.twist.twist.linear.y = y_ / dt;
			odomMsg.twist.twist.angular.z = yaw_ / dt;

			#ifdef MESSAGEOUT_TESTING
			std::cout << std::endl << "x: " << odomMsg.twist.twist.linear.x 
			  << std::endl << "y: " << odomMsg.twist.twist.linear.y << std::endl
			  << "yaw: " << odomMsg.twist.twist.angular.z << std::endl;
			#endif

			// publish odom message, to be consumed by robot_localization
			this->odom_publisher->publish(odomMsg);
		});


	} // constructor

private:
	// member variables
	rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>
												::SharedPtr odom_publisher;
	rclcpp::Subscription<interface::msg::Wheeltravel>::SharedPtr wt_subscriber;

	// body position deltas
	double x_;
	double y_;
	double yaw_;

	// helper function prototypes
	double getPositionX(const interface::msg::Wheeltravel &wt);
	double getPositionY(const interface::msg::Wheeltravel &wt);
	double getYaw(const interface::msg::Wheeltravel &wt);
};


inline double WheelOdomNode::getPositionX(const interface::msg::Wheeltravel &wt)	{
	return WHEEL_RADIUS * (wt.front_right + wt.front_left 
							+ wt.back_right + wt.back_left) / 4.0;
}

inline double WheelOdomNode::getPositionY(const interface::msg::Wheeltravel &wt)	{
	return WHEEL_RADIUS * (wt.front_left + wt.back_right 
							- wt.front_right - wt.back_left) / 4.0;
}

inline double WheelOdomNode::getYaw(const interface::msg::Wheeltravel &wt)		{
	return WHEEL_RADIUS * (wt.front_right + wt.back_right - wt.front_left 
						- wt.back_left) / (2.0 * (TRACK_WIDTH + WHEELBASE));
}

geometry_msgs::msg::Quaternion yawToQuaternion(double yaw)	{
	geometry_msgs::msg::Quaternion q;
	q.x = 0.0;
	q.y = 0.0;
	q.z = std::sin(yaw / 2.0);
	q.w = std::cos(yaw / 2.0);
	return q;
}

int main(int argc, char** argv)	{
	rclcpp::init(argc, argv);
	auto odomNode = std::make_shared<WheelOdomNode>();
	rclcpp::spin(odomNode);
	rclcpp::shutdown();	
}
