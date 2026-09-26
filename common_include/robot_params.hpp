/*								robot_params.hpp							//
	parameters describing robot confirguration. i am aware this should go in
	the robot's URDF, but tbh it's simpler for me to just put in a header file
	since i'm not bothering with simulation/visualization and wrote controller
	from scratch for this project.

	auth: @dchayto
*/

#include <cmath> // for acos

// should really consider a more central location for this information
inline constexpr double WHEEL_RADIUS	{ (60.0 / 2.0) / 1000.0 };		// [m]
inline constexpr double WHEELBASE		{ 0.20 };	// [m]
inline constexpr double TRACK_WIDTH		{ 0.20 };	// [m] - assumes CoM centred
inline constexpr double PI				{ std::acos( -1.0 ) };
inline constexpr double MEC_ANGLE		{ ::PI / 4.0 };	// radians, pos value
inline constexpr double MAX_WHEELSPEED 	{ 20.0 * 1000.0 }; // mrad/s - update when have better idea
