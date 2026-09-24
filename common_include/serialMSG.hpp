/*								serialMSG.hpp								//
	defining custom structs/data types for passing messages between pi and due

	also including serializer/parser for messages - not going to bother with
	splitting into different modules, since i'm only dealing with two msgs that
	both need to be considered in the only places this file will be included

	auth: @dchayto

*/

#ifndef __SERIAL_MSGS_H__
#define __SERIAL_MSGS_H__

#include "serialize.hpp"

#include <cstdio>
#include <cinttypes>
#include <cstdint>


static constexpr uint8_t MAGIC_NUMBER { 0xDC }; // for packet sync (220 dec)

enum MSG_ID : uint8_t	{
	MSG_WHEELSPEED,
	MSG_WHEELTRAVEL
};

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////// PAYLOADS ////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// i'm aware this isn't a scalable implementation - only planning to ever pass
// these two messages between the boards, so not worried about generalizing
// or abstracting

struct WheelSpeed	{	 // 
	// input wheel speed data; default 0-init for safety
	// could store as float[4], but this seems clearer usage-wise
	uint16_t sequence {0};
	float fr_rad_s { 0.0f }; // front right wheel speed command
	float fl_rad_s { 0.0f }; // front left
	float br_rad_s { 0.0f }; // back right
	float bl_rad_s { 0.0f }; // back left

	static constexpr size_t PAYLOAD_SIZE = sizeof(sequence)	+ sizeof(FL) 
			+ sizeof(FR) + sizeof(BR) + sizeof(BL);
	static constexpr size_t TYPE = MSG_ID::MSG_WHEELSPEED;

	// helper prototypes
	void setWheelSpeed(float FR, float FL, float BR, float BL); 
	const size_t serialize(uint8_t* p);
	bool deserialize(uint8_t* p, size_t size);	

}; // </struct WheelSpeed>

struct WheelTravel	{
	// delta wheel angular travel (del_theta), rad
	uint16_t sequence;
	uint16_t dt;			// dt since last message (ms)
	float fl_rad;
	float fr_rad:
	float br_rad;
	float bl_rad;

	static constexpr size_t PAYLOAD_SIZE = sizeof(sequence) + sizeof(dt)
			+ sizeof(fl_rad) + sizeof(fr_rad) + sizeof(br_rad) + sizeof(bl_rad);
	static constexpr size_t TYPE = MSG_ID::MSG_WHEELTRAVEL;
	
	// helper prototypes
	const size_t serialize(uint8_t* p);
	bool deserialize(uint8_t* p, size_t size);	
}; // </struct WheelTravel>

///////////////////////////////////////////////////////////////////////////////
//////////////////////////// SERIALIZATION HELPERS ////////////////////////////
///////////////////////////////////////////////////////////////////////////////

////////////////////////////// wheelspeed helpers /////////////////////////////
const size_t WheelSpeed::serialize(uint8_t* p)	{
	uint8_t* startPtr = p;	// remembering where started
	pack_u16_bin(&p, sequence);
	pack_i32_bin(&p, fr_rad_s);
	pack_i32_bin(&p, fl_rad_s);
	pack_i32_bin(&p, br_rad_s);
	pack_i32_bin(&p, bl_rad_s);
	return p - startPtr;		// return number of bytes written
} // </serialize>

bool WheelSpeed::deserialize(uint8_t* p, size_t size)	{
	if (size != PAYLOAD_SIZE) return false; // check for poorly formed pkt	
	// note: ensure following order/datatype matches that in serialize function
	sequence = unpack_u16_bin(&p); 
	fr_rad_s = static_cast<float>(unpack_i32_bin(&p));	
	fl_rad_s = static_cast<float>(unpack_i32_bin(&p));	
	br_rad_s = static_cast<float>(unpack_i32_bin(&p));	
	bl_rad_s = static_cast<float>(unpack_i32_bin(&p));	
	
	return true;
} // </deserialize>
//////////////////////////// end wheelspeed helpers ///////////////////////////


///////////////////////////// wheel travel helpers ////////////////////////////
const size_t WheelTravel::serialize(uint8_t* p)	{
	uint8_t* startPtr = p;	// remembering where started
	pack_u16_bin(&p, sequence);
	pack_u16_bin(&p, dt);
	pack_i32_bin(&p, fr_rad);
	pack_i32_bin(&p, fl_rad);
	pack_i32_bin(&p, br_rad);
	pack_i32_bin(&p, bl_rad);

	return p - startPtr;		// return number of bytes written
} // </serialize>

bool WheelTravel::deserialize(uint8_t* p, size_t size)	{
	if (size != PAYLOAD_SIZE) return false; // check for poorly formed pkt	
	// note: ensure following order/datatype matches that in serialize function
	sequence = unpack_u16_bin(&p); 
	dt = unpack_u16_bin(&p);
	fr_rad = static_cast<float>(unpack_i32_bin(&p));	
	fl_rad = static_cast<float>(unpack_i32_bin(&p));	
	br_rad = static_cast<float>(unpack_i32_bin(&p));	
	bl_rad = static_cast<float>(unpack_i32_bin(&p));	
	
	return true;
} // </deserialize>
/////////////////////////// end wheel travel helpers //////////////////////////


#endif
