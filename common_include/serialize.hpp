/*                              serialize.hpp                               //  
	helper functions for serializing primative/fixed width data types 

	also including CRC-8 algorithm (stole the code for this one hehe)

	NOTE: all functions move the pointer for the buffer they're passed in
	as data is read or written

	NOTE: not checking for valid pointer on this level - trusting valid pointer
	passed in

    auth: @dchayto                                                              
                                                                                   
 */    

#pragma once 		// being lazy

#include <cstdint>
#include <cstddef>

void pack_u8_bin(uint8_t* &buf, const uint8_t d)	{
	*buf++ = d;
}

uint8_t unpack_u8_bin(uint8_t* &buf)	{
	// trusting compiler to optimize away temp var
	return *buf++;
}

void pack_u16_bin(uint8_t* &buf, const uint16_t d)	{
	*buf++ = static_cast<uint8_t>(d);
	*buf++ = static_cast<uint8_t>(d >> 8);
}

uint16_t unpack_u16_bin(uint8_t* &buf)	{
//	// trusting compiler to optimize away temp var
//	uint16_t val = static_cast<uint16_t>(buf[0])
//		| (static_cast<uint16_t>(buf[1]) << 8); 
//		
//	buf += sizeof(uint16_t);		// advance 2 bytes in msg pointer
//	return val;
	return static_cast<int32_t>(*buf++)
		| (static_cast<int32_t>(*buf++) << 8);
}

void pack_i32_bin(uint8_t* &buf, const int32_t d)	{
	// derefing buff then post increment to move ptr
	*buf++ = static_cast<uint8_t>(d);
	*buf++ = static_cast<uint8_t>(d >> 8);
	*buf++ = static_cast<uint8_t>(d >> 16);
	*buf++ = static_cast<uint8_t>(d >> 24);
}

int32_t unpack_i32_bin(uint8_t* &buf)	{
//	it's probably not actually worth "optimizing" this but what the hell	
//	int32_t val = static_cast<int32_t>(buf[0])
//		| (static_cast<int32_t>(buf[1]) << 8)
//	 	| (static_cast<int32_t>(buf[2]) << 16)
//		| (static_cast<int32_t>(buf[3]) << 24);
//
//	buf += sizeof(int32_t);		// advance 4 bytes
//	
//	return val; 	// return unpacked int32_t
	
	return static_cast<int32_t>(*buf++)
		| (static_cast<int32_t>(*buf++) << 8)
	 	| (static_cast<int32_t>(*buf++) << 16)
		| (static_cast<int32_t>(*buf++) << 24);

}

uint8_t crc8(const uint8_t* data, size_t length)	{
	// stolen from chatcbd - not sure it's the most efficient (dbl for loop...)
	uint8_t crc = 0x00;
	
	for (size_t i = 0; i < length; ++i)	{
		crc ^= data[i];
		for (int bit = 0; bit < 8; ++bit)	{
			if (crc & 0x80)		{ crc = (crc << 1) ^ 0x07; }
			else	crc <<= 1;
		}
	}

	return crc;
}
