/*
 * c_hdl_packet_parser.h
 *
 *  Created on: Mar 14, 2022
 *      Author: amyznikov
 */

#ifndef __c_hdl_packet_parser_h__
#define __c_hdl_packet_parser_h__

#include "c_lidar_frame.h"
#include "c_hdl_lidar_specifcation.h"
#include <inttypes.h>
#include <string>
#include <vector>
#include <memory>
#include <arpa/inet.h>
#include <netinet/in.h>

constexpr int HDL_DATA_BLOCKS_PER_PKT = 12;
constexpr int HDL_LASERS_PER_DATA_BLOCK = 32;



/* in host byte order */
enum HDLLaserBlockID
{
  HDL_BLOCK_00_31 = 0xEEFF,
  HDL_BLOCK_32_63 = 0xDDFF,
  HDL_BLOCK_64_95 = 0xCCFF,
  HDL_BLOCK_96_127 = 0xBBFF,
};


//
//enum HDL_ExtDataPacketMode
//{
//  HDL_EXT_MODE_NONE = -1,
//  HDL_EXT_MODE_TRIPLE_RETURN = 0,
//  HDL_EXT_MODE_CONFIDENCE = 1,
//};
//
//enum HDL_PowerMode
//{
//  HDL_CorrectionOff = 0,
//  HDL_NO_INTERNAL_CORRECTION_0 = 0xa0,
//  HDL_NO_INTERNAL_CORRECTION_1 = 0xa1,
//  HDL_NO_INTERNAL_CORRECTION_2 = 0xa2,
//  HDL_NO_INTERNAL_CORRECTION_3 = 0xa3,
//  HDL_NO_INTERNAL_CORRECTION_4 = 0xa4,
//  HDL_NO_INTERNAL_CORRECTION_5 = 0xa5,
//  HDL_NO_INTERNAL_CORRECTION_6 = 0xa6,
//  HDL_NO_INTERNAL_CORRECTION_7 = 0xa7,
//  HDL_CorrectionOn = 0xa8,
//};
//
//enum HDL_DualFlag
//{
//  HDL_DUAL_DISTANCE_NEAR = 0x1,  // point with lesser distance
//  HDL_DUAL_DISTANCE_FAR = 0x2,   // point with greater distance
//  HDL_DUAL_INTENSITY_HIGH = 0x4, // point with lesser intensity
//  HDL_DUAL_INTENSITY_LOW = 0x8,  // point with greater intensity
//  HDL_DUAL_DOUBLED = 0xf,        // point is single return
//  HDL_DUAL_DISTANCE_MASK = 0x3,
//  HDL_DUAL_INTENSITY_MASK = 0xc,
//};

#pragma pack(push, 1)

struct HDLLaserReturn
{
  uint16_t distance;
  uint8_t intensity;
};

struct HDLDataBlock
{
  uint16_t blockId;
  uint16_t azimuth;
  HDLLaserReturn laserReturns[HDL_LASERS_PER_DATA_BLOCK];
};

struct HDLDataPacket
{
  HDLDataBlock dataBlocks[HDL_DATA_BLOCKS_PER_PKT];
  uint32_t TohTimestamp;
  uint8_t factoryBytes[2];
};

#pragma pack(pop)


class c_hdl_packet_parser
{
public:
  bool parse(const uint8_t * data, uint size);
  void reset();

  HDLSensorType sensor_type() const;
  HDLReturnMode return_mode() const;
  const c_hdl_lidar_specifcation * lidar_specification() const;

  void set_hdl_framing_mode(enum HDLFramingMode v);
  enum HDLFramingMode hdl_framing_mode() const;

  int last_known_azimuth() const;
  int pktcounter() const;

  std::vector<c_lidar_frame::sptr> frames;

protected:
  bool setup(HDLSensorType sensor_type, HDLReturnMode return_mode);
  bool precompute_timing_offsets();
  bool parse_vlp16(const HDLDataPacket *dataPacket);
  bool parse_vlp32(const HDLDataPacket *dataPacket);
  bool parse_hdl32(const HDLDataPacket *dataPacket);
  bool parse_hdl64(const HDLDataPacket *dataPacket);
  bool parse_vls128(const HDLDataPacket *dataPacket);

protected:
  //HDLSensorType sensor_type_ = HDLSensor_unknown;
  HDLReturnMode return_mode_ = HDLReturnMode_unknown;
  HDLFramingMode hdl_framing_mode_ = HDLFraming_Rotation;
  int last_known_azimuth_ = 0;
  int pktcounter_ = 0;

  // current lidar specification table
  c_hdl_lidar_specifcation lidar_specification_;

  // timing offset lookup table
  std::vector< std::vector<float> > timing_offsets_;
  // Caches the azimuth percent offset for the VLS-128 laser firings
  double vls_128_laser_azimuth_cache[16];
};



/** Data-Packet Specifications says that laser-packets are 1206 byte long.
 *  That is : (2+2+(2+1)*32)*12 + 4 + 1 + 1
 *                #lasers^   ^#firingPerPkt
 **/
inline constexpr uint hdl_lidar_packet_size()
{
  return 1206;
}

/**
 * get_sensor_type()
 *
 *  For HDL64 check the sequence of block identifiers, see HDL-64E_S3.pdf Appendix E: Data Packet Format.
 *  For the rest use the factoryField2, see VLP32CManual.pdf Table 9-1 Factory Byte Values.
 */
inline HDLSensorType get_sensor_type(const HDLDataPacket & packet)
{
  const int blockid1 = packet.dataBlocks[1].blockId;
  const int blockid2 = packet.dataBlocks[2].blockId;
  if( blockid1 == HDL_BLOCK_32_63 && blockid2 == HDL_BLOCK_00_31 ) {
    return HDLSensor_HDL64;
  }
  return static_cast<HDLSensorType>(packet.factoryBytes[1]);
}

/**
 * get_return_mode()
 *
 *  For HDL64 check the sequence of block identifiers, see HDL-64E_S3.pdf Appendix E: Data Packet Format.
 *  For the rest use the factoryField1, see VLP32CManual.pdf Table 9-1 Factory Byte Values.
 */
inline HDLReturnMode get_return_mode(const HDLDataPacket & packet)
{
  const HDLSensorType sensor_type =
      get_sensor_type(packet);

  if( sensor_type == HDLSensor_HDL64 ) {
    return (packet.dataBlocks[2].azimuth == packet.dataBlocks[0].azimuth) ?
        HDL_DUAL_RETURN : HDL_STRONGEST_RETURN;
  }

  return static_cast<HDLReturnMode>(packet.factoryBytes[0]);
}


/**
 * is_single_return_mode()
 */
inline bool is_single_return_mode(HDLReturnMode return_mode)
{
  switch (return_mode) {
  case HDL_STRONGEST_RETURN:
    case HDL_LAST_RETURN:
    return true;
  }
  return false;
}


//
//inline constexpr uint hdl_lidar_packet_size()
//{
//  // Data-Packet Specifications says that laser-packets are 1206 byte long.
//  //  That is : (2+2+(2+1)*32)*12 + 4 + 1 + 1
//  //                #lasers^   ^#firingPerPkt
//  return 1206;
//}
//
//inline bool is_hdl_lidar_packet(const uint8_t * data, uint size)
//{
//  if (size == hdl_lidar_packet_size()) {
//    return true;
//  }
//  return false;
//}
//
//inline bool isUpperBlock(const HDL_FiringData & firing)
//{
//  return firing.blockIdentifier == HDL_BLOCK_32_TO_63;
//}
//
//inline uint16_t getElevation100th(const HDL_FiringData & )
//{
//  return 0;
//}
//
//inline int getScanningVerticalDir(const HDL_FiringData & firing)
//{
//  return firing.blockIdentifier >> 15;
//}
//
//inline int getScanningHorizontalDir(const HDL_FiringData & firing)
//{
//  return firing.rotationalPosition >> 15;
//}
//
//inline uint16_t getRotationalPosition(const HDL_FiringData & firing)
//{
//  return firing.rotationalPosition;
//}
//
//inline bool isHDL64(const HDLDataPacket & packet)
//{
//  return packet.firingData[1].blockIdentifier == HDL_BLOCK_32_TO_63 &&
//      packet.firingData[2].blockIdentifier == HDL_BLOCK_0_TO_31;
//}
//
//inline bool isVLS128(const HDLDataPacket & packet)
//{
//  return (packet.factoryField2 == HDLSensor_VLS128 && !isHDL64(packet));
//}
//
//inline HDLSensorType getSensorType(const HDLDataPacket & packet)
//{
//  return isHDL64(packet) ?
//      HDLSensor_HDL64 : static_cast<HDLSensorType>(packet.factoryField2);
//}
//
//inline bool isDualModeReturnHDL64(const HDLDataPacket & packet)
//{
//  return getRotationalPosition(packet.firingData[2]) == getRotationalPosition(packet.firingData[0]);
//}
//
//inline HDL_DualReturnSensorMode getDualReturnSensorMode(const HDLDataPacket & packet)
//{
//  if (isHDL64(packet)) {
//    return isDualModeReturnHDL64(packet) ? HDL_DUAL_RETURN : HDL_STRONGEST_RETURN;
//  }
//
//  return static_cast<HDL_DualReturnSensorMode>(packet.factoryField1);
//}
//
//inline bool hdl_isValidPacket(const unsigned char * data, unsigned int dataLength)
//{
//  if( dataLength != hdl_lidar_packet_size() ) {
//    return false;
//  }
//
//  const HDLDataPacket *dataPacket =
//      reinterpret_cast<const HDLDataPacket*>(data);
//
//  return ((dataPacket->firingData[0].blockIdentifier == HDL_BLOCK_0_TO_31) ||
//      (dataPacket->firingData[0].blockIdentifier == HDL_BLOCK_32_TO_63));
//}
//
//inline bool isDualModeReturnVLS128(const HDLDataPacket & packet)
//{
//  return ((packet.factoryField1 == HDL_DUAL_RETURN) ||
//      (packet.factoryField1 == HDL_DUAL_RETURN_WITH_CONFIDENCE) ||
//      (packet.factoryField1 == HDL_TRIPLE_RETURN));
//}
//
//inline bool isDualModeReturn16Or32(const HDLDataPacket & packet)
//{
//  return getRotationalPosition(packet.firingData[1]) == getRotationalPosition(packet.firingData[0]);
//}
//
//inline bool isDualModeReturn(const HDLDataPacket & packet)
//{
//  if (isVLS128(packet))
//    return isDualModeReturnVLS128(packet);
//  if (isHDL64(packet))
//    return isDualModeReturnHDL64(packet);
//  else
//    return isDualModeReturn16Or32(packet);
//}
//
//
//
//inline int getExtDataPacketType(const HDLDataPacket & packet)
//{
//  if( packet.factoryField1 == HDL_TRIPLE_RETURN ) {
//    return HDL_EXT_MODE_TRIPLE_RETURN;
//  }
//
//  if( packet.factoryField1 == HDL_DUAL_RETURN_WITH_CONFIDENCE ) {
//    return HDL_EXT_MODE_CONFIDENCE;
//  }
//
//  return HDL_EXT_MODE_NONE;
//}
//
//inline bool isDualBlockOfDualPacket128(const int firingBlock)
//{
//  return (firingBlock % 2 == 1);
//}
//
//inline bool isDualBlockOfDualPacket64(const int firingBlock)
//{
//  return (firingBlock % 4 >= 2);
//}
//
//inline bool isDualBlockOfDualPacket16Or32(const int firingBlock)
//{
//  return (firingBlock % 2 == 1);
//}
//
//inline bool isDualReturnFiringBlock(const HDLDataPacket & packet, const int firingBlock)
//{
//  if( isVLS128(packet) ) {
//
//    if( getExtDataPacketType(packet) == HDL_EXT_MODE_NONE ) {
//      return isDualModeReturnVLS128(packet) && isDualBlockOfDualPacket128(firingBlock);
//    }
//
//    if( isDualModeReturnVLS128(packet) ) {
//      if( ((firingBlock == 1) || (firingBlock == 4) || (firingBlock == 7) || (firingBlock == 10)) ) {
//        return true;
//      }
//    }
//
//    return false;
//  }
//
//  if( isHDL64(packet) ) {
//    return isDualModeReturnHDL64(packet) && isDualBlockOfDualPacket64(firingBlock);
//  }
//
//  return isDualModeReturn16Or32(packet) && isDualBlockOfDualPacket16Or32(firingBlock);
//}
//
//
//inline int getRotationalDiffForVLS128(const HDLDataPacket & packet, int firingBlock, int LastAzimuth)
//{
//  if( static_cast<HDL_DualReturnSensorMode>(packet.factoryField1) == HDL_DUAL_RETURN ||
//      static_cast<HDL_DualReturnSensorMode>(packet.factoryField1) == HDL_TRIPLE_RETURN ||
//      static_cast<HDL_DualReturnSensorMode>(packet.factoryField1) == HDL_DUAL_RETURN_WITH_CONFIDENCE ) {
//
//    return static_cast<int>((36000 +
//        packet.firingData[firingBlock].rotationalPosition - LastAzimuth) % 36000);
//  }
//
//  if( firingBlock > 11 - 4 ) {
//    firingBlock = 7;
//  }
//
//  return static_cast<int>((36000 +
//      packet.firingData[firingBlock + 4].rotationalPosition -
//      packet.firingData[firingBlock].rotationalPosition) %
//      36000);
//}
//
//inline int unsignedAngleDiffTo_m180_180deg(uint16_t end_100thDeg, uint16_t start_100thDeg)
//{
//  return static_cast<int>(((36000 + 18000) + end_100thDeg - start_100thDeg) % 36000) - 18000;
//}


//
//  std::string to_tsv_string() const
//  {
//    char sep = '\t';
//    std::stringstream ss;
//    for (int f = 0; f < HDL_FIRING_PER_PKT; f++)
//      ss << "blkIden:" << sep << std::hex << firingData[f].blockIdentifier << sep;
//    ss << std::endl;
//    for (int f = 0; f < HDL_FIRING_PER_PKT; f++)
//      ss << "blkAzm:" << sep << std::dec << firingData[f].rotationalPosition << sep;
//    ss << std::endl;
//    for (int f = 0; f < HDL_FIRING_PER_PKT; f++)
//      ss << "rawD" << sep << "intens" << sep;
//    ss << std::endl;
//    for (int dsr = 0; dsr < HDL_LASER_PER_FIRING; dsr++)
//    {
//      for (int f = 0; f < HDL_FIRING_PER_PKT; f++)
//      {
//        ss << (int)firingData[f].laserReturns[dsr].distance << sep
//           << (int)firingData[f].laserReturns[dsr].intensity << sep;
//      }
//      ss << std::endl;
//    }
//    for (int f = 0; f < HDL_FIRING_PER_PKT - 2; f++)
//      ss << sep << sep;
//    ss << "gpsTime:" << sep << (int)TohTimestamp << sep;
//    ss << std::endl;
//    for (int f = 0; f < HDL_FIRING_PER_PKT - 2; f++)
//      ss << sep << sep;
//
//    ss << "factyField1:" << sep << "0x" << std::hex << (int)factoryField1 << sep;
//    ss << std::endl;
//    for (int f = 0; f < HDL_FIRING_PER_PKT - 2; f++)
//      ss << sep << sep;
//    ss << "factyField2:" << sep << "0x" << std::hex << (int)factoryField2 << sep;
//    ss << std::endl;
//    return ss.str();
//  }

#endif /* __c_hdl_packet_parser_h__ */
