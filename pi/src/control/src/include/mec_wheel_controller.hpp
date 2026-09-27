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

