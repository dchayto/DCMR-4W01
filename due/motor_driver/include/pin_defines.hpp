// 								pin_defines.hpp								//
// probably going to want to change a lot of these when wiring is done
// motor PWM signals
const int FR_PWM = 2; 	// GPIO 2 (PWM)
const int FL_PWM = 3;	// GPIO 3 (PWM)
const int BR_PWM = 4;	// GPIO 4 (PWM)
const int BL_PWM = 7;	// GPIO 7 (PWM)
// GPIOs 5 and 6 apparently have weird interactions w/ other timer functions

// half-bridge controls (fwd/reverse)
const int FR_REV = 22;	// GPIO 22
const int FL_REV = 24;	// GPIO 24
const int BR_REV = 26;	// GPIO 26
const int BL_REV = 28;	// GPIO 28

// encoder signals (receive)
const int FR_ENC = 23;	// GPIO 23
const int FL_ENC = 25;	// GPIO 25
const int BR_ENC = 27;	// GPIO 27
const int BL_ENC = 29;	// GPIO 29

// ultrasonic sensor signals


