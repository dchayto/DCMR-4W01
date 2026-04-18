// 								pin_defines.hpp								//
// probably going to want to change a lot of these when wiring is done

// NOTE: FR is driver 1 motor A,  FL is driver 1 motor B,  BR -> 2B, BL -> 2A

// motor PWM signals
const int FR_PWM = 2; 	// GPIO 2 (PWM)
const int FL_PWM = 3;	// GPIO 3 (PWM)
const int BR_PWM = 4;	// GPIO 4 (PWM)
const int BL_PWM = 7;	// GPIO 7 (PWM)
// GPIOs 5 and 6 apparently have weird interactions w/ other timer functions

// half-bridge controls (fwd/reverse)
// NOTE: wired flipped left-right, since driving direction relative to wheel
// direction is inverted
const int FR_FWD = 23;	// 1A, in1 (PURPLE)	
const int FR_REV = 25;	// 1A, in2 (GREY)	

const int FL_FWD = 24;	// 1B, in4 (BLACK)	
const int FL_REV = 22;	// 1B, in3 (WHITE)	

const int BR_FWD = 46;	// 2B, in3 (WHITE)
const int BR_REV = 44;	// 2B, in4 (BLACK)	

const int BL_FWD = 45;	// 2A, in2 (GREY) 
const int BL_REV = 47;	// 2A, in1 (PURPLE)	


// encoder signals (receive)
const int FR_ENCA = 100;	// SET LATER
const int FR_ENCB = 100;	// SET LATER

const int FL_ENCA = 100;	// SET LATER
const int FL_ENCB = 100;	// SET LATER

const int BR_ENCA = 100;	// SET LATER
const int BR_ENCB = 100;	// SET LATER

const int BL_ENCA = 100;	// SET LATER
const int BL_ENCB = 100;	// SET LATER

// ultrasonic sensor signals
// SET LATER


