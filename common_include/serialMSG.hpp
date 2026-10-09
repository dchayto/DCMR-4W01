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

#undef TESTING

#include "serialize.hpp"

#include <cstdint>

#ifdef TESTING 
#include <iostream> 
#endif

namespace serialMSG	{

enum MSG_ID : uint8_t	{
	MSG_WHEELSPEED,
	MSG_WHEELTRAVEL
};

constexpr uint16_t MAGIC_NUMBER { 0xDC26 }; // for packet sync
static constexpr uint8_t FRAMING_SIZE = static_cast<uint8_t>(
	sizeof(MAGIC_NUMBER) 	+ 
	sizeof(MSG_ID) 			+
	sizeof(uint8_t)			+	// seq number - don't love this being a mn
	sizeof(decltype(crc8(nullptr, 0)))	// size of crc8
);


///////////////////////////////////////////////////////////////////////////////
//////////////////////////////// PAYLOADS ////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// i'm aware this isn't a scalable implementation - only planning to ever pass
// these two messages between the boards, so not worried about generalizing
// or abstracting

struct WheelSpeed	{
	// input wheel speed data; default 0-init for safety
	// could store as double[4], but this seems clearer usage-wise
	double fr_rad_s { 0.0 }; // front right wheel speed command
	double fl_rad_s { 0.0 }; // front left
	double br_rad_s { 0.0 }; // back right
	double bl_rad_s { 0.0 }; // back left

	// wire format of payload is 4x int32_t
	static constexpr uint8_t PAYLOAD_SIZE = static_cast<uint8_t>(4 * sizeof(int32_t));
	static constexpr uint8_t MSG_SIZE = PAYLOAD_SIZE + FRAMING_SIZE;
	static constexpr uint8_t TYPE = MSG_ID::MSG_WHEELSPEED;

	// helper prototypes
	size_t serialize(uint8_t* &p);
	bool deserialize(uint8_t* p);	

}; // </struct WheelSpeed>

struct WheelTravel	{
	// delta wheel angular travel (del_theta), rad
	// i know putting uint16 is bad for struct packing, but seemed more
	// use-intuitive - shouldn't affect wire size anyways
	uint16_t dt;			// dt since last message (ms)
	double fr_rad;
	double fl_rad;
	double br_rad;
	double bl_rad;

	// wire format of payload is 1 uint16_t + 4x int32_t
	static constexpr uint8_t PAYLOAD_SIZE = static_cast<uint8_t>(
		sizeof(uint16_t) + 4 * sizeof(int32_t));
	static constexpr uint8_t MSG_SIZE = PAYLOAD_SIZE + FRAMING_SIZE;
	static constexpr uint8_t TYPE = MSG_ID::MSG_WHEELTRAVEL;
	
	// helper prototypes
	size_t serialize(uint8_t* &p);
	bool deserialize(uint8_t* p);	
}; // </struct WheelTravel>

///////////////////////////////////////////////////////////////////////////////
//////////////////////////// SERIALIZATION HELPERS ////////////////////////////
///////////////////////////////////////////////////////////////////////////////

////////////////////////////// wheelspeed helpers /////////////////////////////
size_t WheelSpeed::serialize(uint8_t* &p)	{
	if (p == nullptr) 	return 0;

	uint8_t* startPtr = p;	// remembering where started
	pack_i32_bin(p, static_cast<int32_t>(fr_rad_s*1000.0));
	pack_i32_bin(p, static_cast<int32_t>(fl_rad_s*1000.0));
	pack_i32_bin(p, static_cast<int32_t>(br_rad_s*1000.0));
	pack_i32_bin(p, static_cast<int32_t>(bl_rad_s*1000.0));
	return p - startPtr;		// return number of bytes written
} // </serialize>

bool WheelSpeed::deserialize(uint8_t* p)	{
	if (p == nullptr) 	return false;

	// note: ensure following order/datatype matches that in serialize function
	fr_rad_s = static_cast<double>(unpack_i32_bin(p))/1000.0;	
	fl_rad_s = static_cast<double>(unpack_i32_bin(p))/1000.0;	
	br_rad_s = static_cast<double>(unpack_i32_bin(p))/1000.0;	
	bl_rad_s = static_cast<double>(unpack_i32_bin(p))/1000.0;	

	return true;
	
//	return true;
} // </deserialize>
//////////////////////////// end wheelspeed helpers ///////////////////////////


///////////////////////////// wheel travel helpers ////////////////////////////
size_t WheelTravel::serialize(uint8_t* &p)	{
	if (p == nullptr) 	return 0;

	uint8_t* startPtr = p;	// remembering where started
	pack_u16_bin(p, dt);
	pack_i32_bin(p, static_cast<int32_t>(fr_rad*1000));
	pack_i32_bin(p, static_cast<int32_t>(fl_rad*1000));
	pack_i32_bin(p, static_cast<int32_t>(br_rad*1000));
	pack_i32_bin(p, static_cast<int32_t>(bl_rad*1000));

	return p - startPtr;		// return number of bytes written
} // </serialize>

bool WheelTravel::deserialize(uint8_t* p)	{
	if (p == nullptr)	return false;

	// note: ensure following order/datatype matches that in serialize function
	dt = unpack_u16_bin(p);
	fr_rad = static_cast<double>(unpack_i32_bin(p))/1000.0;	
	fl_rad = static_cast<double>(unpack_i32_bin(p))/1000.0;	
	br_rad = static_cast<double>(unpack_i32_bin(p))/1000.0;	
	bl_rad = static_cast<double>(unpack_i32_bin(p))/1000.0;	

	return true;
} // </deserialize>
/////////////////////////// end wheel travel helpers //////////////////////////


/////////////////////////// full packet serialization /////////////////////////
template <typename T>
size_t serializePacket(uint8_t seq, uint8_t* const buf, T& payload)	{
	// reminder: [MAGIC_NUMBER][TYPE][SEQUENCE][PAYLOAD][CRC-8]
	if (buf == nullptr)		return 0;

	uint8_t* p = buf;

	// write packet:
	pack_u16_bin(p, MAGIC_NUMBER);	
	pack_u8_bin(p, payload.TYPE);
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
uint8_t* parsePacket(uint8_t& seq, uint8_t* const packet)	{
	// reminder: [MAGIC_NUMBER][TYPE][SEQUENCE][PAYLOAD][CRC-8]
	if (packet == nullptr)	return nullptr;

	uint8_t* p = packet;	
	if (unpack_u16_bin(p) != MAGIC_NUMBER) return nullptr;	// invalid start
	
	// read in rest of header 
	uint8_t type = unpack_u8_bin(p);
	uint8_t temp_seq = unpack_u8_bin(p);		// assign seq to return parameter
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
	else {
		seq = temp_seq;
		return payloadPtr;
	}
}
/////////////////////////////// end packet parser /////////////////////////////

}


#ifdef TESTING
int main()	{
	using namespace serialMSG;

	// define example wheeltravel message, print basic info about message
	WheelSpeed ws { 1.0, 1.0, 0.0, 1.0 };
	WheelSpeed ws2 { };
	std::cout << "WheelTravel message (type " << +ws.TYPE 
		<< "): { FR: " << ws.fr_rad_s 
		<< ", FL: " << ws.fl_rad_s << " BR: " << ws.br_rad_s
		<< " BL: " << ws.bl_rad_s << " }" << std::endl;
	std::cout << "Framing size (computed separately): " << +FRAMING_SIZE << std::endl;
	std::cout << "Payload size (computed manually): " << +ws.PAYLOAD_SIZE << std::endl;
	std::cout << "Message size (payload + framing): " << +ws.MSG_SIZE << std::endl;
	
	// attempt to encode ws message
	uint8_t buffer [64];
	size_t write_size = serializePacket(1, buffer, ws);	
	std::cout << std::endl << "Bytes written: " << write_size 
		<< std::endl << "Buffer contents:";
	for (size_t i = 0; i < write_size; ++i)	{
		std::cout << " " << +buffer[i];
	}
	std::cout << std::endl;

	// attempt to decode ws message into ws2
	uint8_t seq { 1 };
	uint8_t* payloadPtr = parsePacket(seq, buffer); 
	if (payloadPtr == nullptr)	{
		std::cout << "Could not decode packet. An issue exists either with "
			<< "packet encoding or with parser." << std::endl;
	} else	{
		std::cout << "Payload parsed to be at idx " << +(payloadPtr - buffer) << std::endl;
		ws2.deserialize(payloadPtr);
		std::cout << "Decoded message: { FR: " << ws2.fr_rad_s 
			<< ", FL: " << ws2.fl_rad_s << " BR: " << ws2.br_rad_s
			<< " BL: " << ws2.bl_rad_s << " } " << std::endl;
	}

	ws.fr_rad_s = 3.2;
	ws.fl_rad_s = 3.2;
	ws.br_rad_s = 3.2;
	ws.bl_rad_s = 3.2;

	write_size = serializePacket(1, buffer, ws);	
	std::cout << std::endl << "Bytes written: " << write_size 
		<< std::endl << "Buffer contents:";
	for (size_t i = 0; i < write_size; ++i)	{
		std::cout << " " << +buffer[i];
	}
	std::cout << std::endl;

	// attempt to decode wheelspeed message
	payloadPtr = parsePacket(seq, buffer); 
	if (payloadPtr == nullptr)	{
		std::cout << "Could not decode packet. An issue exists either with "
			<< "packet encoding or with parser." << std::endl;
	} else	{
		std::cout << "Payload parsed to be at idx " << +(payloadPtr - buffer) << std::endl;
		ws2.deserialize(payloadPtr);
		std::cout << "Decoded message: { " 
			<< "FR: " << ws2.fr_rad_s 
			<< ", FL: " << ws2.fl_rad_s << " BR: " << ws2.br_rad_s
			<< " BL: " << ws2.bl_rad_s << " } " << std::endl;
	}

	return 0;
}
#endif

#endif
