/*							mec_wheel_controller.hpp						//
	helper functions, variable declarations, structs definitions for mech wheel
	controller

	auth: @dchayto
*/

#include <cmath>

// should really consider a more central location for this information
inline constexpr double WHEEL_RADIUS	{ (60.0 / 2.0) / 1000.0 };		// [m]
inline constexpr double WHEELBASE		{ 0.20 };	// [m]
inline constexpr double TRACK_WIDTH		{ 0.20 };	// [m] - assumes CoM centred
inline constexpr double PI				{ std::acos( -1.0 ) };
inline constexpr double MEC_ANGLE		{ ::PI / 4.0 };	// radians, pos value
inline constexpr double MAX_WHEELSPEED 	{ 20.0 }; // rad/s - update when have better idea

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

