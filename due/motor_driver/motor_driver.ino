/*							motor_driver.ino								//
	arduino sketch to handle various low-level tasks for driving motors on
	robot. (handles sending PWM commands, listening for encoder messages,
	basic processing + formatting of encoder messages before sending upstream)

	should listen for messages from pi, process messages (send control sig to
	motors), read and process encoder data, and pass processed encoder data
	back to pi for higher-level controls use

	auth: @dchayto
*/

#include <cstdint>

#include "include/pin_defines.hpp"
#include "include/cmd_flag.hpp"
#include "include/encoder.hpp"

#include "serialMSG.hpp"
#include "PID.hpp"

using namespace serialMSG;

static WheelSpeed ws_rad_s { };
static WheelTravel wt_rad { };
static uint8_t MOTOR_PWM[4] { 0, 0, 0, 0 }; // FR, FL, BR, BL
static int8_t DDIR[4] { 1, 1, 1, 1 }; 	// 1 for fwd, 0 for bkwd

#define DRIVE_ENABLE		// for enabling/disabling PWM commands

#define MESSAGEIN_TESTING		// for testing message recieving
#define MESSAGEOUT_TESTING		// for testing message passing
#undef MOTOR_TESTING			// for testing motor hardware

inline void drive()	{
	// drive pins based on current status of control vars (PWM, DDIR)
	#ifndef DRIVE_ENABLE
		return;
	#endif

	// ul min pwm to turn was 72; not really run in yet, so setting a bit lower
	static constexpr uint8_t MOTOR_DEADZONE { 64 };
	for (int i = 0; i < 4; ++i)	{
		if (MOTOR_PWM[i] < MOTOR_DEADZONE) MOTOR_PWM[i] = 0;
	}

	digitalWrite(FR_FWD, DDIR[0]);
	digitalWrite(FR_REV, 1 - DDIR[0]);
	analogWrite(FR_PWM, MOTOR_PWM[0]);

	digitalWrite(FL_FWD, DDIR[1]);
	digitalWrite(FL_REV, 1 - DDIR[1]);
	analogWrite(FL_PWM, MOTOR_PWM[1]);
	
	digitalWrite(BR_FWD, DDIR[2]);
	digitalWrite(BR_REV, 1 - DDIR[2]);
	analogWrite(BR_PWM, MOTOR_PWM[2]);

	digitalWrite(BL_FWD, DDIR[3]);
	digitalWrite(BL_REV, 1 - DDIR[3]);
	analogWrite(BL_PWM, MOTOR_PWM[3]);
}

inline void stop()	{
	for (int i = 0; i < 4; ++i)	{
		MOTOR_PWM[i] = 0;
	}
	// needs to work even if drive_enable is disabled
	analogWrite(FR_PWM, MOTOR_PWM[0]);
	analogWrite(FL_PWM, MOTOR_PWM[1]);
	analogWrite(BR_PWM, MOTOR_PWM[2]);
	analogWrite(BL_PWM, MOTOR_PWM[3]);
}

void setup() {
	// set pins to safe state, initialize as req'd (set as input/output, etc.)
	pinMode(FR_FWD, OUTPUT);	digitalWrite(FR_FWD, 0);
	pinMode(FR_REV, OUTPUT);	digitalWrite(FR_REV, 0);
	pinMode(FR_PWM, OUTPUT);	analogWrite(FR_PWM, 0);
	
	pinMode(FL_FWD, OUTPUT);	digitalWrite(FL_FWD, 0);
	pinMode(FL_REV, OUTPUT);	digitalWrite(FL_REV, 0);
	pinMode(FL_PWM, OUTPUT);	analogWrite(FL_PWM, 0);
	
	pinMode(BR_FWD, OUTPUT);	digitalWrite(BR_FWD, 0);
	pinMode(BR_REV, OUTPUT);	digitalWrite(BR_REV, 0);
	pinMode(BR_PWM, OUTPUT);	analogWrite(BR_PWM, 0);
	
	pinMode(BL_FWD, OUTPUT);	digitalWrite(BL_FWD, 0);
	pinMode(BL_REV, OUTPUT);	digitalWrite(BL_REV, 0);
	pinMode(BL_PWM, OUTPUT);	analogWrite(BL_PWM, 0);

	stop();		// pull pwm pins low just in case lol
	initEncoders();
	
	// configure serial port
	Serial.begin(57600); 	// ensure this matches baud rate on pi
	
	// clear input buffer
	while (Serial.available()) Serial.read();

	#ifdef MESSAGEIN_TESTING
	// send ack message through serial port
	Serial.println("due online");
	if (Serial.available()) Serial.println("data available in serial buffer");
	else Serial.println("no data available in serial buffer");
	#endif

}

#ifndef MOTOR_TESTING
void loop() { 
	// SAFETY TIMEOUT ON MOTORS
	static unsigned long motorTimer { millis() };
	static constexpr unsigned long MOTOR_TIMEOUT { 1500 }; // time out after 1.5s
	if ((millis() - motorTimer) > MOTOR_TIMEOUT)	{
		ws_rad_s.fr_rad_s = 0.0;
		ws_rad_s.fl_rad_s = 0.0;
		ws_rad_s.br_rad_s = 0.0;
		ws_rad_s.bl_rad_s = 0.0;
		drive();
	}


	// NOTE: arduino serial buffer is 64 byte
	// NOTE: ring buffer MUST be power of 2 for bitwise math to work
	// NOTE: nomenclature here is a bit confusing: searching backwards through
	//		 ring buffer, so using head as most recent element, tail as oldest
	static constexpr uint8_t BUFFER_SIZE { 128 }; // enough for 2 full serial buffers
	static uint8_t input_buffer[BUFFER_SIZE];	// ring buffer for serial data 
	static uint8_t head { 0 };	// head (write) for buffer)
	static uint8_t tail { 0 };	// tail (read) for buffer

 	// if data in system buffer, read into ring buffer 
	while (Serial.available())	{ 
		input_buffer[head] = Serial.read();		// read in a byte	
		++head &= (BUFFER_SIZE - 1); 			// handle wraparound
	}

	// if have a full string available in ring buffer (head is >= MSG_SIZE
	// pos ahead of tail), look for message between head-MSG_SIZE and tail
	// NOTE: not bothering with sequence number for wheelspeed messages, since
	// only processing latest message, which is handled by head/tail
	if ((head >= tail ? head - tail : head + BUFFER_SIZE - tail) 
												>= WheelSpeed::MSG_SIZE)	{
		// look bkwds thru input buf 4 start of frame, stop if tail reached
		uint8_t wsSeq;	// don't care, but need smth to pass in to parser

		uint8_t startpos = (head - WheelSpeed::MSG_SIZE) & (BUFFER_SIZE - 1);

		// loop through until reaching tail (location of last parsed msg)
		for (uint8_t searchidx = startpos; 
								searchidx != ((tail - 1) & (BUFFER_SIZE - 1)); 
											--searchidx &= (BUFFER_SIZE - 1))	{

			// copy message into message buffer
			// there's probably a better way to do this, but can't be at a
			// wrapover when going to serialMSG functions, so storing in sep buf
			static uint8_t message_buffer[32];
			for (size_t i = 0; i < WheelSpeed::MSG_SIZE; ++i)	{
				message_buffer[i] = input_buffer[(searchidx+i) & (BUFFER_SIZE - 1)];
			}

			uint8_t* msgStart = parsePacket(wsSeq, message_buffer);

			if (msgStart != nullptr)	{
				ws_rad_s.deserialize(msgStart);
				CMD_FLAG |= WHEELCMD_RECEIVED;	// note that command recieved
				break;	// stop loop early; found start of message	
			}
		} // end of buffer parsing
		tail = startpos;	// set tail to latest parsed value
	} // end of message received conditional 


	{	// scope definition for PIDs
	static constexpr double p_wheel { 50.0 };
	static constexpr double i_wheel { 1.0 };
	static constexpr double d_wheel { 0.0 };

	static PID frPID { p_wheel, i_wheel, d_wheel };
	static PID flPID { p_wheel, i_wheel, d_wheel };
	static PID brPID { p_wheel, i_wheel, d_wheel };
	static PID blPID { p_wheel, i_wheel, d_wheel };
	
	// PID loop
	static unsigned long t0PID = micros();
	static constexpr unsigned long PID_TIMER { 2000 };	// 500 Hz
	unsigned long dtPID = micros() - t0PID; 
	if (dtPID >= PID_TIMER) {
		double dt_s = static_cast<double>(dtPID) * 1e-6;	// convert micros to s

		double u_fr = ENC_TO_RAD(fr_enc_count) / dt_s;
		double u_fl = ENC_TO_RAD(fl_enc_count) / dt_s;
		double u_br = ENC_TO_RAD(br_enc_count) / dt_s;
		double u_bl = ENC_TO_RAD(bl_enc_count) / dt_s;

		double e_fr = frPID.correct(ws_rad_s.fr_rad_s - u_fr, dt_s);
		double e_fl = flPID.correct(ws_rad_s.fl_rad_s - u_fl, dt_s);
		double e_br = brPID.correct(ws_rad_s.br_rad_s - u_br, dt_s);
		double e_bl = blPID.correct(ws_rad_s.bl_rad_s - u_bl, dt_s);

		auto generatePWM = [](double error, uint8_t& mPWM, int8_t& dDir)	{
			if (error >= 0)	{
				dDir = 1;
			} else	{
				dDir = 0;
				error = -error;
			}
			mPWM = (error < 255.0 ? static_cast<uint8_t>(error) : 255);
		};
		generatePWM(e_fr, MOTOR_PWM[0], DDIR[0]);
		generatePWM(e_fl, MOTOR_PWM[1], DDIR[1]);
		generatePWM(e_br, MOTOR_PWM[2], DDIR[2]);
		generatePWM(e_bl, MOTOR_PWM[3], DDIR[3]);
			
		drive();
	}

	// if command to wheels received, process now
	if (CMD_FLAG & WHEELCMD_RECEIVED)	{
	
		CMD_FLAG &= ~WHEELCMD_RECEIVED;	// unset flag
		motorTimer = millis();			// reset timer
		tail = head;					// only process msg once

		// if new command receieved, reset PID controllers 
		static WheelSpeed ws_prev { };
		static constexpr double TH_CMD { 0.025 };	// threshold on what new cmd means
		
		if (abs(ws_prev.fr_rad_s - ws_rad_s.fr_rad_s) > TH_CMD)	frPID.reset();	
		if (abs(ws_prev.fl_rad_s - ws_rad_s.fl_rad_s) > TH_CMD)	flPID.reset();	
		if (abs(ws_prev.br_rad_s - ws_rad_s.br_rad_s) > TH_CMD)	brPID.reset();	
		if (abs(ws_prev.bl_rad_s - ws_rad_s.bl_rad_s) > TH_CMD)	blPID.reset();	

		ws_prev.fr_rad_s = ws_rad_s.fr_rad_s;
		ws_prev.fl_rad_s = ws_rad_s.fl_rad_s;
		ws_prev.br_rad_s = ws_rad_s.br_rad_s;
		ws_prev.bl_rad_s = ws_rad_s.bl_rad_s;

		#ifdef MESSAGEIN_TESTING
		Serial.println("MESSAGE RECEIVED: ");
		Serial.print("RECEIVED: {FR: ");
		Serial.print(ws_rad_s.fr_rad_s);
		Serial.print("}  {FL: ");
		Serial.print(ws_rad_s.fl_rad_s);
		Serial.print("}  {BR: ");
		Serial.print(ws_rad_s.br_rad_s);
		Serial.print("}  {BL: ");
		Serial.print(ws_rad_s.bl_rad_s);
		Serial.println("}");
		#endif
	}
	} // scope definition for PIDs

	// read encoder data
	// NOTE: ensure encoder timer is larger than timer on wheeltravel
	// publisher in due_hw_interface (might miss messages otherwise)
	static constexpr unsigned long ENCODER_TIMER { 10000 }; 	// 100hz freq
	static uint8_t encoder_buffer[32];
	static uint8_t enc_seq { 0 };
	static unsigned long timeOfLastSend { micros() };
	// send encoder message
	if ((micros() - timeOfLastSend) > ENCODER_TIMER)	{
		wt_rad.fr_rad = ENC_TO_RAD(fr_enc_count);
		wt_rad.fl_rad = ENC_TO_RAD(fl_enc_count);
		wt_rad.br_rad = ENC_TO_RAD(br_enc_count);
		wt_rad.bl_rad = ENC_TO_RAD(bl_enc_count);
		wt_rad.dt = (micros() - timeOfLastSend) / 1000;

		// serialize and write message
		size_t bytes_written = serializePacket(enc_seq, encoder_buffer, wt_rad);
		if (bytes_written == WheelTravel::MSG_SIZE)	{
			Serial.write(encoder_buffer, bytes_written);
			timeOfLastSend = micros();	// reset timer
			resetEncoder();
			++enc_seq;
		}
	}

} // </loop>
#endif


// TESTING MOTOR CONTROLS
#ifdef MOTOR_TESTING
void loop()	{
	// testing forward controls
	stop();
	delay(10000);	// looking for dead zones - delay for time to set up tio
	for (int i = 0; i < 4; ++i)	{
		DDIR[i] = 1;
//		for (int k = 0; k < 256; k += 16)	{
		for (int k = 64; k < 200; k += 8)	{
			MOTOR_PWM[i] = k;
			drive();
//			delay(500);
			Serial.print("MOTOR: ");
			Serial.print(i);
			Serial.print("\tPWM: ");
			Serial.println(k);
			delay(1500);
		}
		stop();
//		for (int k = 255; k > 0; k -= 16)	{
//			MOTOR_PWM[i] = k;
//			drive();
//			delay(500);
//		}
	}
	
	delay(1500);
	stop();

	// testing reverse controls	
	for (int i = 0; i < 4; ++i)	{
		DDIR[i] = 0;
//		for (int k = 0; k < 256; k += 16)	{
		for (int k = 64; k < 200; k += 8)	{
			MOTOR_PWM[i] = k;
			drive();
//			delay(500);
			Serial.print("MOTOR: ");
			Serial.print(i);
			Serial.print("\tPWM: ");
			Serial.println(k);
			delay(1500);
		}
		stop();
//		for (int k = 255; k > 0; k -= 16)	{
//			MOTOR_PWM[i] = k;
//			drive();
//			delay(500);
//		}
	}
	delay(1500);
}
#endif
