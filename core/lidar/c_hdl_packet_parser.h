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

  void set_hdl_framing_mode(enum HDLFramingMode v);
  enum HDLFramingMode hdl_framing_mode() const;

  void set_hdl_frame_seam_azimuth(double azimuth_in_degrees);
  double hdl_frame_seam_azimuth() const;

  HDLSensorType sensor_type() const;
  HDLReturnMode return_mode() const;
  const c_hdl_lidar_specifcation * lidar_specification() const;


  int last_known_azimuth() const;
  int pktcounter() const;

  std::vector<c_lidar_frame::sptr> frames;

protected:
  bool setup(HDLSensorType sensor_type, HDLReturnMode return_mode);
  bool precompute_correction_tables();
  bool parse_vlp16(const HDLDataPacket *dataPacket);
  bool parse_vlp32(const HDLDataPacket *dataPacket);
  bool parse_hdl32(const HDLDataPacket *dataPacket);
  bool parse_hdl64(const HDLDataPacket *dataPacket);
  bool parse_vls128(const HDLDataPacket *dataPacket);
  bool is_hdl_frame_seam(int current_packet_azimuth, int previous_packet_azimuth) const;


protected:
  HDLReturnMode return_mode_ = HDLReturnMode_unknown;
  HDLFramingMode hdl_framing_mode_ = HDLFraming_Rotation;
  int last_known_azimuth_ = 0;
  int pktcounter_ = 0;
  double hdl_frame_seam_azimuth_ = 0;

  // current lidar specification table
  c_hdl_lidar_specifcation lidar_specification_;

  struct c_lasers_corrections_table {
    double sin_rot_correction;
    double cos_rot_correction;
    double sin_vert_correction;
    double cos_vert_correction;
  };

  // precomputed coordinate corrections table
  std::vector<c_lasers_corrections_table> precomuted_corrections_table_;

  // precomputed timing offset lookup table
  std::vector< std::vector<float> > precomputed_timing_offsets_;

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

#endif /* __c_hdl_packet_parser_h__ */
