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
#include "UARTmsgs.hpp"

static UARTmsgs::WheelSpeed ws { };
static uint8_t MOTOR_PWM[4] { 0, 0, 0, 0 };
static int8_t DDIR[4] { 1, 1, 1, 1 }; 	// 1 for fwd, -1 for bkwd
static const int ENCODER_TIMER { 1000 }; 	// 1 sec freq for sending enc data

inline void drive()	{
	// drive pins based on current status of control vars
	analogWrite(FR_PWM, MOTOR_PWM[0]);
	analogWrite(FR_REV, DDIR[0]);

	analogWrite(FR_PWM, MOTOR_PWM[1]);
	analogWrite(FR_REV, DDIR[1]);

	analogWrite(FR_PWM, MOTOR_PWM[2]);
	analogWrite(FR_REV, DDIR[2]);
	
	analogWrite(FR_PWM, MOTOR_PWM[3]);
	analogWrite(FR_REV, DDIR[3]);
}

void generatePWM()	{
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

void adjustPWM()	{
	// check to see if encoder ratios match input command ratio (ws)

	// if not, apply gentle correction to PWM lines

}

void setup() {
	// configure UART port
	Serial.begin(57600); 	// ensure this matches baud rate on pi
	
	// set pins to safe state, initialize as req'd (set as input/output, etc.)
	pinMode(FR_REV, OUTPUT);
	pinMode(FL_REV, OUTPUT);
	pinMode(BR_REV, OUTPUT);
	pinMode(BL_REV, OUTPUT);

	pinMode(FR_ENC, INPUT);
	pinMode(FL_ENC, INPUT);
	pinMode(BR_ENC, INPUT);
	pinMode(BL_ENC, INPUT);

}

void loop() {
	/*
	// TESTING BASIC COMMS - DELETE AFTERWARDS	
	char i {};
	if (Serial.available())	{
		Serial.readBytes(&i, 1);	
		Serial.write(&i, 1);	
	}
	*/

	// check for serial data on UART port
	if (Serial.available() >= ws.MSG_SIZE)	{
		if (Serial.readBytes(ws.msg, ws.MSG_SIZE) == ws.MSG_SIZE)	{
			ws.decodeMsg();
			CMD_FLAG |= WHEELCMD_RECEIVED;	// note that command recieved
			Serial.write(ws.msg, UARTmsgs::WheelSpeed::MSG_SIZE);
		}
	}

	// if command to wheels received, handle now
	if (CMD_FLAG & WHEELCMD_RECEIVED)	{
		// send command to motors
		//drive(); // UNCOMMENT LATER

		// testing message passing - delete afterwards
		//Serial.write(ws.msg, UARTmsgs::WheelSpeed::MSG_SIZE);

		// unset flag
		CMD_FLAG |= WHEELCMD_RECEIVED;	
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

	// scheduling sending data
	static long timeOfLastSend { millis() };
	if ((millis() - timeOfLastSend) > ENCODER_TIMER)	{
		timeOfLastSend = millis();	// reset timer
		//Serial.write(buffer, length);
		// for now, just going to send a simple string back and forth
	}
	*/	

}
