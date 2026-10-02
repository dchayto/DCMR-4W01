/* 									encoder.hpp								//
					encoder parameters and helper functions
	
	auth: @dchayto
*/

#include "pin_defines.hpp"	// already should be linked thru motor_driver, safety

constexpr int PULSES_PER_TURN { 960 };	// 8 pulses per turn * 120 GR 

volatile long fr_enc_count { 0 };
volatile long fl_enc_count { 0 };
volatile long br_enc_count { 0 };
volatile long bl_enc_count { 0 };

inline constexpr double ENC_TO_MRAD(long encoder_count)	{
	return (1000.0 * encoder_count / PULSES_PER_TURN) / TWO_PI;
}

void frEncOnPulse()	{
	// ccw positive direction
	if (FR_ENCA == FR_ENCB)		++fr_enc_count;
	else						--fr_enc_count;
}
void flEncOnPulse()	{
	// cw positive direction
	if (FL_ENCA != FL_ENCB)		++fl_enc_count;
	else						--fl_enc_count;
}
void brEncOnPulse()	{
	// ccw positive direction
	if (BR_ENCA == BR_ENCB)		++br_enc_count;
	else						--br_enc_count;
}
void blEncOnPulse()	{
	// cw positive direction
	if (BL_ENCA != BL_ENCB)		++bl_enc_count;
	else						--bl_enc_count;
}

void resetEncoder()	{
	fr_enc_count = 0;
	fl_enc_count = 0;
	br_enc_count = 0;
	bl_enc_count = 0;
}

void initEncoders() {
	// note: use encoder B (second pin) for interrupt (motor datasheet says so)
	pinMode(FR_ENCA, INPUT);
	pinMode(FR_ENCB, INPUT);
	attachInterrupt(digitalPinToInterrupt(FR_ENCB, frEncOnPulse, CHANGE);

	pinMode(FL_ENCA, INPUT);
	pinMode(FL_ENCB, INPUT);
	attachInterrupt(digitalPinToInterrupt(FL_ENCB, flEncOnPulse, CHANGE);

	pinMode(BR_ENCA, INPUT);
	pinMode(BR_ENCB, INPUT);
	attachInterrupt(digitalPinToInterrupt(BR_ENCB, brEncOnPulse, CHANGE);

	pinMode(BL_ENCA, INPUT);
	pinMode(BL_ENCB, INPUT);
	attachInterrupt(digitalPinToInterrupt(BL_ENCB, blEncOnPulse, CHANGE);
}

