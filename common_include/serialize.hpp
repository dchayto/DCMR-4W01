/*                              serialize.hpp                               //  
	helper functions for serializing primative/fixed width data types 

	NOTE: all functions move the pointer for the buffer they're passed in
	as data is read or written

    auth: @dchayto                                                              
                                                                                   
 */    

#pragma once 		// being lazy

#include <cstdint>
#include <cstdstring>

constexpr void pack_u16_bin(uint8_t* buf, const uint16_t d)	{
	*buf++ = static_cast<uint8_t>(d);
	*buf++ = static_cast<uint8_t>(d >> 8);
}

constexpr uint16_t unpack_u16_bin(const uint8_t* buf)	{
	// trusting compiler to optimize away temp var
	uint16_t val = static_cast<uint16_t>(buf[0])
		| (static_cast<uint16_t>(buf[1]) << 8); 
		
	buf += sizeof(uint16_t);		// advance 2 bytes in msg pointer
	return val;
}

constexpr void pack_i32_bin(uint8_t* buf, const int32_t d)	{
	// derefing buff then post increment to move ptr
	*buf++ = static_cast<uint8_t>(d);
	*buf++ = static_cast<uint8_t>(d >> 8);
	*buf++ = static_cast<uint8_t>(d >> 16);
	*buf++ = static_cast<uint8_t>(d >> 24);
}

constexpr int32_t unpack_i32_bin(const uint8_t* buf)	{
	int32_t static_cast<int32_t>(buf[0])
		| (static_cast<int32_t>(buf[1]) << 8);
	 	| (static_cast<int32_t>(buf[2]) << 16);
		| (static_cast<int32_t>(buf[3]) << 24);

	buf += sizeof(int32_t);		// advance 4 bytes
	return val; 	// return unpacked int32_t
}
