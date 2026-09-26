/*							mec_wheel_controller.hpp						//
	helper functions, variable declarations, structs definitions for mech wheel
	controller

	auth: @dchayto
*/

// could really just use an array, but might be nice to be able to refer
// to as "twist.x" instead of twist[0]
struct twist	{
	double x {0.0};		// linear x velocity [m/s]
	double y {0.0};		// linear y velocity [m/s]
	double w {0.0};		// angular z velocity [rad/s]	

	double getLength() const	{
		return std::sqrt(std::pow(x, 2.0) + std::pow(y, 2.0) + std::pow(w, 2.0));
	}
};

// using output from controller, calculate wheelspeeds based on kin model
//
// IGNORING TRANSFORMS TO WORLD FRAME - going to handle any world:body 
// transforms on world-side system (i.e., use robot pose to transform
// desired global position/velocity command to appropriate body frame command
// i.e., considering body frame to "be" global frame
//
// NOTE: considering:
//			- driving direction as x 
//			- vertical up as z
//			- left as y (by right hand naming convention)
//			- theta(world->body) as 0rad
//			- thata(body->wheel) as 0rad (driving direction of wheel frame 
// 			  parallel to driving direction of robot frame
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
// 		v_drive = (1/r)*v_x + (g)*v_y + (x_wheel*g - y_wheel*g)*w_z
//  where g = tan(MEC_ANGLE) / r
//
// 	NEED TO UPDATE THIS CODE TO USE STATIC TF2 PUBLISHER
double MecWheelControllerNode::getFrontRightWS()	{
	// remember to use +MEC_ANGLE, +X_WHEEL, -Y_WHEEL
	static const double G 			{ std::tan(+MEC_ANGLE) / WHEEL_RADIUS };
	static const double X_WHEEL 	{ +WHEELBASE / 2.0 };
	static const double Y_WHEEL 	{ -TRACK_WIDTH / 2.0 };
	static const double G_X 		{ 1.0 / WHEEL_RADIUS };
	// static const double G_Y 		{ G };
	static const double G_W			{ X_WHEEL*G - Y_WHEEL*G };

	static double v_drive;
	v_drive = G_X * ctrlTwist.x + G * ctrlTwist.y + G_W * ctrlTwist.w;
	return v_drive / WHEEL_RADIUS;
}

double MecWheelControllerNode::getFrontLeftWS()		{
	// remember to use -MEC_ANGLE, +X_WHEEL, +Y_WHEEL
	static const double G 			{ std::tan(-MEC_ANGLE) / WHEEL_RADIUS };
	static const double X_WHEEL 	{ +WHEELBASE / 2.0 };
	static const double Y_WHEEL 	{ +TRACK_WIDTH / 2.0 };
	static const double G_X 		{ 1.0 / WHEEL_RADIUS };
	// static const double G_Y 		{ G };
	static const double G_W			{ X_WHEEL*G - Y_WHEEL*G };

	static double v_drive;
	v_drive = G_X * ctrlTwist.x + G * ctrlTwist.y + G_W * ctrlTwist.w;
	return v_drive / WHEEL_RADIUS;
}

double MecWheelControllerNode::getBackRightWS()		{
	// remember to use -MEC_ANGLE, -X_WHEEL, -Y_WHEEL
	static const double G 			{ std::tan(-MEC_ANGLE) / WHEEL_RADIUS };
	static const double X_WHEEL 	{ -WHEELBASE / 2.0 };
	static const double Y_WHEEL 	{ -TRACK_WIDTH / 2.0 };
	static const double G_X 		{ 1.0 / WHEEL_RADIUS };
	// static const double G_Y 		{ G };
	static const double G_W			{ X_WHEEL*G - Y_WHEEL*G };

	static double v_drive;
	v_drive = G_X * ctrlTwist.x + G * ctrlTwist.y + G_W * ctrlTwist.w;
	return v_drive / WHEEL_RADIUS;
}

double MecWheelControllerNode::getBackLeftWS()		{
	// remember to use +MEC_ANGLE, -X_WHEEL, +Y_WHEEL
	static const double G 			{ std::tan(+MEC_ANGLE) / WHEEL_RADIUS };
	static const double X_WHEEL 	{ -WHEELBASE / 2.0 };
	static const double Y_WHEEL 	{ +TRACK_WIDTH / 2.0 };
	static const double G_X 		{ 1.0 / WHEEL_RADIUS };
	// static const double G_Y 		{ G };
	static const double G_W			{ X_WHEEL*G - Y_WHEEL*G };

	static double v_drive;
	v_drive = G_X * ctrlTwist.x + G * ctrlTwist.y + G_W * ctrlTwist.w;
	return v_drive / WHEEL_RADIUS;
}

