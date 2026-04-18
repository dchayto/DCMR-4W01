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

static serialMSG::WheelSpeed ws { };
static uint8_t MOTOR_PWM[4] { 0, 0, 0, 0 }; // FR, FL, BR, BL
static int8_t DDIR[4] { 1, 1, 1, 1 }; 	// 1 for fwd, 0 for bkwd

#define MESSAGE_TESTING		// for testing message passing/recieving
#define DRIVE_ENABLE		// for enabling/disabling PWM commands
#undef MOTOR_TESTING

inline void drive()	{
	// for now, just print wheelspeed commands to console
	// drive pins based on current status of control vars
	generatePWM();
	
	#ifndef DRIVE_ENABLE
		return;
	#endif

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
	for (int i = 0; i < 4; ++i)	MOTOR_PWM[i] = 0;
	drive();
}

inline void generatePWM()	{
	// send pwm commands based on current input ws, state of PWM array, and 
	// reverse flags, taking old PWM array for starting point

	// what this should be doing is turning the PWM inputs and drive direction
	// into [-256 256] linear scale, doing math based on this scale, then re-
	// converting into [0 256] PWM input with a driving direction to write


	// MAKE SURE TO RAMP INPUTS
	// probably ok to just make any given input change take the same number
	// of cycles (as opposed to e.g., just inc by 1 for everything until gets
	// to desired number)
	// -> can do in 8 cycles by taking step as (num2 - num1) >> 3; however,
	// this makes anything with a gap of less than 8 happen instantly on last
	// step (should be making last item the final step anyways to handle
	// rounding errors)

}

void setup() {
	// configure serial port
	Serial.begin(57600); 	// ensure this matches baud rate on pi
	
	// clear input buffer
	while (Serial.available()) Serial.read();
	
	// set pins to safe state, initialize as req'd (set as input/output, etc.)
	pinMode(FR_FWD, OUTPUT);	digitalWrite(FR_FWD, 0);
	pinMode(FR_REV, OUTPUT);	digitalWrite(FR_REV, 0);
	pinMode(FR_PWM, OUTPUT);	analogWrite(FR_PWM, 0);
//	pinMode(FR_ENCA, INPUT);
//	pinMode(FR_ENCB, INPUT);
	
	pinMode(FL_FWD, OUTPUT);	digitalWrite(FL_FWD, 0);
	pinMode(FL_REV, OUTPUT);	digitalWrite(FL_REV, 0);
	pinMode(FL_PWM, OUTPUT);	analogWrite(FL_PWM, 0);
//	pinMode(FL_ENCA, INPUT);
//	pinMode(FL_ENCB, INPUT);
	
	pinMode(BR_FWD, OUTPUT);	digitalWrite(BR_FWD, 0);
	pinMode(BR_REV, OUTPUT);	digitalWrite(BR_REV, 0);
	pinMode(BR_PWM, OUTPUT);	analogWrite(BR_PWM, 0);
//	pinMode(BR_ENCA, INPUT);
//	pinMode(BR_ENCB, INPUT);
	
	pinMode(BL_FWD, OUTPUT);	digitalWrite(BL_FWD, 0);
	pinMode(BL_REV, OUTPUT);	digitalWrite(BL_REV, 0);
	pinMode(BL_PWM, OUTPUT);	analogWrite(BL_PWM, 0);
//	pinMode(BL_ENCA, INPUT);
//	pinMode(BL_ENCB, INPUT);
}

// TESTING MOTOR CONTROLS
#ifdef MOTOR_TESTING
void loop()	{
	// testing forward controls
	stop();

	for (int i = 0; i < 4; ++i)	{
		DDIR[i] = 1;
		for (int k = 32; k < 256; k += 32)	{
			MOTOR_PWM[i] = k;
			drive();
			delay(250);
		}
	}
	
	delay(1000);
	stop();

	// testing reverse controls	
	for (int i = 0; i < 4; ++i)	{
		DDIR[i] = 0;
		for (int k = 32; k < 256; k += 32)	{
			MOTOR_PWM[i] = k;
			drive();
			delay(250);
		}
	}
	delay(1000);
}
#endif
#ifndef MOTOR_TESTING
void loop() { 
	// SAFETY TIMEOUT ON MOTORS
	static unsigned long motorTimer { millis() };
	static constexpr unsigned long MOTOR_TIMEOUT { 1500 }; // time out after 1.5s
	if ((millis() - motorTimer) > MOTOR_TIMEOUT)	{
		ws.setWheelSpeed(0, 0, 0, 0);
		drive();
	}


	using namespace serialMSG;

	// NOTE: arduino serial buffer is 64 byte
	// NOTE: ring buffer MUST be power of 2 for bitwise math to work
	static constexpr uint8_t BUFFER_SIZE { 32 }; // enough for 3 messages
	static char input_buffer[BUFFER_SIZE];	// ring buffer for serial data 
	static uint8_t head { 0 };	// head (write) for buffer)
	static uint8_t tail { 0 };	// tail (read) for buffer

 	// if data in system buffer, read into ring buffer 
	while (Serial.available())	{ 
		input_buffer[head] = Serial.read();	
		++head &= (BUFFER_SIZE - 1); // head+=1-(head%BUFFER_SIZE) [0, bufsize-1]
	}

	// if have a full string available in ring buffer (head is >= ws.MSG_SIZE
	// pos ahead of tail), look for message between head-MSG_SIZE and tail
	if ((head > tail ? head - tail : head + BUFFER_SIZE - tail) > ws.MSG_SIZE)	{
		// starting at HEAD - ws.MSG_SIZE (first possible location for start of 
		// valid full frame), look backwards through input buffer for start of
		// frame (i.e., check for magic number), stopping if tail reached
		static uint8_t searchpos;
		// start at position of head less one message
		searchpos = (head - WheelSpeed::MSG_SIZE) & (BUFFER_SIZE - 1);
		for (uint8_t searchidx {searchpos}, timeout {0}
				; timeout < WheelSpeed::MSG_SIZE; ++timeout)	{
			if (searchidx == tail) break; // MAKE SURE NOT GOING PAST TAIL
			if (input_buffer[searchidx] == WheelSpeed::MAGIC_NUMBER)	{
				// magic number found; start of message
				// read MSG_SIZE worth of bytes from ring buf to ws.msg; process message
				for (uint8_t copyIdx {0}; copyIdx < WheelSpeed::MSG_SIZE; ++copyIdx)	{
					ws.msg[copyIdx] = input_buffer[searchidx];
					++searchidx &= (BUFFER_SIZE - 1); 
				}
				CMD_FLAG |= WHEELCMD_RECEIVED;	// note that command recieved
				break;	// stop loop early; found start of message	
			}	else	{
				// magic number not found; keep looking until end condition
				--searchidx &= BUFFER_SIZE - 1;	// go back 1 position in ring
			}
		}
		tail = searchpos;	// set tail to latest parsed value
	} // end of buffer parsing


	// if command to wheels received, handle now
	if (CMD_FLAG & WHEELCMD_RECEIVED)	{
		ws.decodeMsg();				// process string message into wheelspeeds
		drive();						// send command to motors
		CMD_FLAG &= ~WHEELCMD_RECEIVED;	// unset flag
		motorTimer = millis();			// reset timer
#ifdef MESSAGE_TESTING
		Serial.println(ws.msg); 		// testing message passing
#endif
	}

	/* COMMENTING OUT WHILE TESTING BASIC COMMS
	// read encoder data

	// send encoder data to helper functions for processing (_if_ doing onboard)
		// while same input sending, monitor encoder data and make sure
		// matches intended input - if not, do minor course correction to pwm 

	// should be returning velocity vector generated from encoder,
	// plus distance travelled based on encoder data since last polling
	// (not sure if distance travelled should be calculated here or elsewhere
	// - may just do here)

	static const int ENCODER_TIMER { 1000 }; 	// 1 sec freq for sending enc data
	// scheduling sending data
	static unsigned long timeOfLastSend { millis() };
	if ((millis() - timeOfLastSend) > ENCODER_TIMER)	{
		timeOfLastSend = millis();	// reset timer
		//Serial.write(buffer, length);
		// for now, just going to send a simple string back and forth
	}
	*/	
} // </loop>
#endif
