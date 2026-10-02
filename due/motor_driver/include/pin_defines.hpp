/* 								pin_defines.hpp								//
	pin definitions for due hookups (GPIO array). probably going to want to 
	change a lot of these when wiring is done

	NOTE: FR is driver 1 motor A,  FL is driver 1 motor B,  BR -> 2B, BL -> 2A

	auth: @dchayto
*/

#ifndef DUE_PIN_DEFINES
#define DUE_PIN DEFINES

// motor PWM signals
constexpr int FR_PWM = 2; 	// GPIO 2 (PWM)
constexpr int FL_PWM = 3;	// GPIO 3 (PWM)
constexpr int BR_PWM = 4;	// GPIO 4 (PWM)
constexpr int BL_PWM = 7;	// GPIO 7 (PWM)
// GPIOs 5 and 6 apparently have weird interactions w/ other timer functions

// half-bridge controls (fwd/reverse)
// NOTE: wired flipped left-right, since driving direction relative to wheel
// direction is inverted
constexpr int FR_FWD = 23;	// 1A, in1 (PURPLE)	
constexpr int FR_REV = 25;	// 1A, in2 (GREY)	

constexpr int FL_FWD = 24;	// 1B, in4 (BLACK)	
constexpr int FL_REV = 22;	// 1B, in3 (WHITE)	

constexpr int BR_FWD = 46;	// 2B, in3 (WHITE)
constexpr int BR_REV = 44;	// 2B, in4 (BLACK)	

constexpr int BL_FWD = 45;	// 2A, in2 (GREY) 
constexpr int BL_REV = 47;	// 2A, in1 (PURPLE)	


// encoder signals (receive)
constexpr int FR_ENCA = 100;	// SET LATER
constexpr int FR_ENCB = 100;	// SET LATER

constexpr int FL_ENCA = 100;	// SET LATER
constexpr int FL_ENCB = 100;	// SET LATER

constexpr int BR_ENCA = 100;	// SET LATER
constexpr int BR_ENCB = 100;	// SET LATER

constexpr int BL_ENCA = 100;	// SET LATER
constexpr int BL_ENCB = 100;	// SET LATER

// ultrasonic sensor signals
// SET LATER

#endif

