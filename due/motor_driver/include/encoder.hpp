/* 									encoder.hpp								//
					encoder parameters and helper functions
	
	auth: @dchayto
*/

constexpr int PULSES_PER_TURN { 960 };	// 8 pulses per turn * 120 GR 

volatile long fr_enc_count;
volatile long fl_enc_count;
volatile long br_enc_count;
volatile long bl_enc_count;

inline constexpr double ENC_TO_RAD(long encoder_count)	{
	return (encoder_count / PULSES_PER_TURN) / TWO_PI;
}

void frEncOnPulse()	{
	// ccw positive direction
	if (digitalRead(FR_ENCA) == HIGH)		++fr_enc_count;
	else									--fr_enc_count;
}
void flEncOnPulse()	{
	// cw positive direction
	if (digitalRead(FL_ENCA) != HIGH)		++fl_enc_count;
	else									--fl_enc_count;
}
void brEncOnPulse()	{
	// ccw positive direction
	if (digitalRead(BR_ENCA) == HIGH)		++br_enc_count;
	else									--br_enc_count;
}
void blEncOnPulse()	{
	// cw positive direction
	if (digitalRead(BL_ENCA) != HIGH)		++bl_enc_count;
	else									--bl_enc_count;
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
	attachInterrupt(digitalPinToInterrupt(FR_ENCB), frEncOnPulse, RISING);

	pinMode(FL_ENCA, INPUT);
	pinMode(FL_ENCB, INPUT);
	attachInterrupt(digitalPinToInterrupt(FL_ENCB), flEncOnPulse, RISING);

	pinMode(BR_ENCA, INPUT);
	pinMode(BR_ENCB, INPUT);
	attachInterrupt(digitalPinToInterrupt(BR_ENCB), brEncOnPulse, RISING);

	pinMode(BL_ENCA, INPUT);
	pinMode(BL_ENCB, INPUT);
	attachInterrupt(digitalPinToInterrupt(BL_ENCB), blEncOnPulse, RISING);

	fr_enc_count = 0;
	fl_enc_count = 0;
	br_enc_count = 0;
	bl_enc_count = 0;
}

