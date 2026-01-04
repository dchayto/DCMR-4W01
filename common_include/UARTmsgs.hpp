/*								UARTmsgs.hpp								//
	defining custom structs/data types for passing messages between pi and due

	auth: @dchayto

*/

#ifndef __UART_MSGS_H__
#define __UART_MSGS_H__

#include <cstdio>
#include <cinttypes>
#include <cstdint>

namespace UARTmsgs		{
	struct WheelSpeed	{
		// input wheel speed data
		// could store as int8_t[4], but this seems clearer usage-wise
		int8_t FR { 0 }; // front right wheel speed command
		int8_t FL { 0 }; // front left
		int8_t BR { 0 }; // back right
		int8_t BL { 0 }; // back left
		
		// vars for handling message formatting
		static const int MSG_SIZE { 32 }; // largest string: "-128 -128 -128 -128" [19]
		char msg[MSG_SIZE];
		
		// kind of sucks, because have to remember to call them before sending
		// and after recieving messages, would be nice to abstract a bit more
		void encodeMsg() {
			using namespace std; // arduino defaults snprintf into global ns
			snprintf(msg, MSG_SIZE, "%+" PRId8 
						" %+" PRId8 " %+" PRId8 " %+" PRId8, FR, FL, BR, BL);
		}

		void decodeMsg() {
			std::sscanf(msg, "%hhd %hhd %hhd %hhd" , &FR, &FL, &BR, &BL);
		}
		
	};

} // UART_msgs

#endif
