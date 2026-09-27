/*								serialMSG.hpp								//
	defining custom structs/data types for passing messages between pi and due

	also including serializer/parser for messages - not going to bother with
	splitting into different modules, since i'm only dealing with two msgs that
	both need to be considered in the only places this file will be included

	NOTE: assumes full message is going to be less than 128 bytes. not checking
	to confirm that this isn't the case, since message formats are fairly
	limited + controlled (NOT SURE IF THIS IS ACTUALLY AN ASSUMPTION ANYMORE)

	full packet is [MAGIC_NUMBER][TYPE][SEQUENCE][PAYLOAD][CRC-8]
	                  uint_16    uint_8  uint_8   varies   uint_8

	auth: @dchayto

*/

#ifndef __SERIAL_MSGS_H__
#define __SERIAL_MSGS_H__

#include "serialize.hpp"

#include <cstdint>

namespace serialMSG	{

enum MSG_ID : uint8_t	{
	MSG_WHEELSPEED,
	MSG_WHEELTRAVEL
};

constexpr uint16_t MAGIC_NUMBER { 0xDCDC }; // for packet sync (56540 dec)
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

struct WheelSpeed	{
	// input wheel speed data; default 0-init for safety
	// could store as double[4], but this seems clearer usage-wise
	double fr_mrad_s { 0.0 }; // front right wheel speed command
	double fl_mrad_s { 0.0 }; // front left
	double br_mrad_s { 0.0 }; // back right
	double bl_mrad_s { 0.0 }; // back left

	// wire format of payload is 4x int32_t
	static constexpr uint8_t PAYLOAD_SIZE = static_cast<uint8_t>(4 * sizeof(int32_t));
	static constexpr uint8_t TYPE = MSG_ID::MSG_WHEELSPEED;

	// helper prototypes
	size_t serialize(uint8_t* p);
	void deserialize(uint8_t* p);	

}; // </struct WheelSpeed>

struct WheelTravel	{
	// delta wheel angular travel (del_theta), mrad
	double fr_mrad;
	double fl_mrad;
	double br_mrad;
	double bl_mrad;
	uint16_t dt;			// dt since last message (ms)

	// wire format of payload is 1 uint16_t + 4x int32_t
	static constexpr uint8_t PAYLOAD_SIZE = static_cast<uint8_t>(
		sizeof(uint16_t) + 4 * sizeof(int32_t));
	static constexpr uint8_t TYPE = MSG_ID::MSG_WHEELTRAVEL;
	
	// helper prototypes
	size_t serialize(uint8_t* p);
	void deserialize(uint8_t* p);	
}; // </struct WheelTravel>

///////////////////////////////////////////////////////////////////////////////
//////////////////////////// SERIALIZATION HELPERS ////////////////////////////
///////////////////////////////////////////////////////////////////////////////

////////////////////////////// wheelspeed helpers /////////////////////////////
size_t WheelSpeed::serialize(uint8_t* p)	{
	uint8_t* startPtr = p;	// remembering where started
	pack_i32_bin(p, static_cast<int32_t>(fr_mrad_s));
	pack_i32_bin(p, static_cast<int32_t>(fl_mrad_s));
	pack_i32_bin(p, static_cast<int32_t>(br_mrad_s));
	pack_i32_bin(p, static_cast<int32_t>(bl_mrad_s));
	return p - startPtr;		// return number of bytes written
} // </serialize>

void WheelSpeed::deserialize(uint8_t* p)	{
//	if (size != PAYLOAD_SIZE) return false; // check for poorly formed pkt	
	// note: ensure following order/datatype matches that in serialize function
	fr_mrad_s = static_cast<double>(unpack_i32_bin(p));	
	fl_mrad_s = static_cast<double>(unpack_i32_bin(p));	
	br_mrad_s = static_cast<double>(unpack_i32_bin(p));	
	bl_mrad_s = static_cast<double>(unpack_i32_bin(p));	
	
//	return true;
} // </deserialize>
//////////////////////////// end wheelspeed helpers ///////////////////////////


///////////////////////////// wheel travel helpers ////////////////////////////
size_t WheelTravel::serialize(uint8_t* p)	{
	uint8_t* startPtr = p;	// remembering where started
	pack_u16_bin(p, dt);
	pack_i32_bin(p, static_cast<int32_t>(fr_mrad));
	pack_i32_bin(p, static_cast<int32_t>(fl_mrad));
	pack_i32_bin(p, static_cast<int32_t>(br_mrad));
	pack_i32_bin(p, static_cast<int32_t>(bl_mrad));

	return p - startPtr;		// return number of bytes written
} // </serialize>

void WheelTravel::deserialize(uint8_t* p)	{
//	if (size != PAYLOAD_SIZE) return false; // check for incomplete pkt	
	// note: ensure following order/datatype matches that in serialize function
	dt = unpack_u16_bin(p);
	fr_mrad = static_cast<double>(unpack_i32_bin(p));	
	fl_mrad = static_cast<double>(unpack_i32_bin(p));	
	br_mrad = static_cast<double>(unpack_i32_bin(p));	
	bl_mrad = static_cast<double>(unpack_i32_bin(p));	
	
//	return true;
} // </deserialize>
/////////////////////////// end wheel travel helpers //////////////////////////


/////////////////////////// full packet serialization /////////////////////////
template <typename T>
size_t serializePacket(uint8_t seq, uint8_t* const buf, T& payload)	{
	uint8_t* p = buf;

	// write packet:
	pack_u16_bin(p, MAGIC_NUMBER);	
	pack_u8_bin(p, payload.TYPE);
	pack_u8_bin(p, payload.PAYLOAD_SIZE);
	pack_u8_bin(p, seq);
	payload.serialize(p);
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
uint8_t* parsePacket(uint8_t& seq, uint8_t* const packet)	{
	uint8_t* p = packet;	
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

}

#endif
