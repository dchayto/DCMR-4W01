/*								serialMSG.hpp								//
	defining custom structs/data types for passing messages between pi and due

	also including serializer/parser for messages - not going to bother with
	splitting into different modules, since i'm only dealing with two msgs that
	both need to be considered in the only places this file will be included

	NOTE: assumes full message is going to be less than 128 bytes. not checking
	to confirm that this isn't the case, since message formats are fairly
	limited + controlled

	full packet is [MAGIC_NUMBER][TYPE][SEQUENCE][PAYLOAD][CRC-8]
	                  uint_16    uint_8  uint_8   varies   uint_8

	auth: @dchayto

*/

#ifndef __SERIAL_MSGS_H__
#define __SERIAL_MSGS_H__

#include "serialize.hpp"

#include <cstdio>
#include <cinttypes>
#include <cstdint>

enum MSG_ID : uint8_t	{
	MSG_WHEELSPEED,
	MSG_WHEELTRAVEL
};

static constexpr uint16_t MAGIC_NUMBER { 0xDCDC }; // for packet sync (56540 dec)
//static constexpr size_t HEADER_SIZE =
//	sizeof(MAGIC_NUMBER) 	+ 
//	sizeof(MSG_ID) 			+
//	sizeof(uint8_t)			+ 	// payload size - don't love the magic number
//	sizeof(uint8_t);	// sequence number - again, don't love this being a mn


///////////////////////////////////////////////////////////////////////////////
//////////////////////////////// PAYLOADS ////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// i'm aware this isn't a scalable implementation - only planning to ever pass
// these two messages between the boards, so not worried about generalizing
// or abstracting

struct WheelSpeed	{	 // 
	// input wheel speed data; default 0-init for safety
	// could store as float[4], but this seems clearer usage-wise
	float fr_rad_s { 0.0f }; // front right wheel speed command
	float fl_rad_s { 0.0f }; // front left
	float br_rad_s { 0.0f }; // back right
	float bl_rad_s { 0.0f }; // back left

	static constexpr uint8_t PAYLOAD_SIZE = static_cast<uint8_t>(sizeof(FL) 
		+ sizeof(FR) + sizeof(BR) + sizeof(BL));
	static constexpr uint8_t TYPE = MSG_ID::MSG_WHEELSPEED;

	// helper prototypes
	const size_t serialize(uint8_t* p);
	bool deserialize(uint8_t* p, size_t size);	

}; // </struct WheelSpeed>

struct WheelTravel	{
	// delta wheel angular travel (del_theta), rad
	uint16_t dt;			// dt since last message (ms)
	float fl_rad;
	float fr_rad:
	float br_rad;
	float bl_rad;

	static constexpr uint8_t PAYLOAD_SIZE = static_cast<uint8_t>(sizeof(dt)
		+ sizeof(fl_rad) + sizeof(fr_rad) + sizeof(br_rad) + sizeof(bl_rad);
	static constexpr uint8_t TYPE = MSG_ID::MSG_WHEELTRAVEL;
	
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
	pack_i32_bin(&p, fr_rad_s);
	pack_i32_bin(&p, fl_rad_s);
	pack_i32_bin(&p, br_rad_s);
	pack_i32_bin(&p, bl_rad_s);
	return p - startPtr;		// return number of bytes written
} // </serialize>

void WheelSpeed::deserialize(uint8_t* p, size_t size)	{
//	if (size != PAYLOAD_SIZE) return false; // check for poorly formed pkt	
	// note: ensure following order/datatype matches that in serialize function
	fr_rad_s = static_cast<float>(unpack_i32_bin(&p));	
	fl_rad_s = static_cast<float>(unpack_i32_bin(&p));	
	br_rad_s = static_cast<float>(unpack_i32_bin(&p));	
	bl_rad_s = static_cast<float>(unpack_i32_bin(&p));	
	
//	return true;
} // </deserialize>
//////////////////////////// end wheelspeed helpers ///////////////////////////


///////////////////////////// wheel travel helpers ////////////////////////////
const size_t WheelTravel::serialize(uint8_t* p)	{
	uint8_t* startPtr = p;	// remembering where started
	pack_u16_bin(&p, dt);
	pack_i32_bin(&p, fr_rad);
	pack_i32_bin(&p, fl_rad);
	pack_i32_bin(&p, br_rad);
	pack_i32_bin(&p, bl_rad);

	return p - startPtr;		// return number of bytes written
} // </serialize>

void WheelTravel::deserialize(uint8_t* p, size_t size)	{
//	if (size != PAYLOAD_SIZE) return false; // check for incomplete pkt	
	// note: ensure following order/datatype matches that in serialize function
	dt = unpack_u16_bin(&p);
	fr_rad = static_cast<float>(unpack_i32_bin(&p));	
	fl_rad = static_cast<float>(unpack_i32_bin(&p));	
	br_rad = static_cast<float>(unpack_i32_bin(&p));	
	bl_rad = static_cast<float>(unpack_i32_bin(&p));	
	
//	return true;
} // </deserialize>
/////////////////////////// end wheel travel helpers //////////////////////////


/////////////////////////// full packet serialization /////////////////////////
template <typename payload>
size_t serializePacket(uint8_t seq, const uint8_t* buf, const &payload msg)	{
	uint8_t* p = buf;

	// write packet:
	pack_u16_bin(p, MAGIC_NUMBER);	
	pack_u8_bin(p, msg::TYPE);
	pack_u8_bin(p, msg::PAYLOAD_SIZE);
	pack_u8_bin(p, seq);
	msg.serialize(p);
	uint8_t crc = crc8(buf, p - buf);	// compute CRC
	pack_u8_bin(p, crc);	

	return p - buf; // return number of bytes in packet
}
///////////////////////// end full packet serialization ///////////////////////

///////////////////////////////// packet parser ///////////////////////////////
// take in candidate packet found in hardware interface buffer (i.e., ID'd MN),
// check to make sure the packet is valid (check CRC), and if valid, return
// the sequence number via the reference parameter, and a pointer to the start
// of the payload via the function parameter (if found)
// reminder: [MAGIC_NUMBER][TYPE][SEQUENCE][PAYLOAD][CRC-8]
uint8_t* parsePacket(uint8_t& seq, const uint8_t* packet)	{
	const uint8_t* p = packet;	
	if (unpack_u16_bin(p) != MAGIC_NUMBER) return nullptr;	// invalid start
	
	// read in rest of header 
	uint8_t type = unpack_u8_bin(p);
	seq = unpack_u16_bin(p);		// assign seq to return parameter
	uint8_t* payloadPtr = p;		// next read is first item in payload

	switch(type)
	{
		case MSG_WHEELSPEED:
			p+= WheelSpeed::PAYLOAD_SIZE;		
			break;

		case MSG_WHEELTRAVEL:
			p+= WheelTravel::PAYLOAD_SIZE;		
			break;
		default:
			return nullptr;		// unrecognized type
			break;
	}

	// check if CRCs match
	if (crc8(packet, p - packet) != unpack_u8_bin(p)) return nullptr;
	else return payloadPtr;
}
/////////////////////////////// end packet parser /////////////////////////////

#endif
