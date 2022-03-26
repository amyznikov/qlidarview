/*
 * c_hdl_packet_parser.cc
 *
 *  Created on: Mar 14, 2022
 *      Author: amyznikov
 */

#include "c_hdl_packet_parser.h"
#include "hdl_lidar_specifcation_db_xml.h"
#include <algorithm>
#include <cmath>
#include <core/ssprintf.h>
#include <core/debug.h>


/** Special Definitions for VLS128 support
 * These are used to detect which bank of 32 lasers is in this block
 * **/
//static constexpr uint16_t VLS128_BANK_1 = 0xeeff;
//static constexpr uint16_t VLS128_BANK_2 = 0xddff;
//static constexpr uint16_t VLS128_BANK_3 = 0xccff;
//static constexpr uint16_t VLS128_BANK_4 = 0xbbff;
static constexpr double  VLS128_CHANNEL_TDURATION =  2.665;  // [µs] Channels corresponds to one laser firing
static constexpr double  VLS128_SEQ_TDURATION     =  53.3;   // [µs] Sequence is a set of laser firings including recharging
static constexpr double  VLS128_TOH_ADJUSTMENT    =  8.7;    // [µs] μs. Top Of the Hour is aligned with the fourth firing group in a firing sequence.



static inline double SQR(double val)
{
  return val * val;
}
//
//bool setup_lasers_table(HDLSensorType sensor_type, c_hdl_lidar_specifcation * spec)
//{
//  spec->lasers.clear();
//
//  const c_hdl_lidar_specifcation * current_specification =
//      get_hdl_lidar_specifcation(sensor_type);
//
//  if( !current_specification ) {
//    CF_ERROR("get_default_hdl_lidar_specifcation(sensor_type=%s (%d)) fails",
//        toString(sensor_type), (int )(sensor_type));
//    return false;
//  }
//
//  * spec = * current_specification;
//
//  return true;
//}
void c_hdl_packet_parser::reset()
{
  frames.clear();
  lidar_specification_.lasers.clear();
  lidar_specification_.sensor = HDLSensor_unknown;
  return_mode_ = HDLReturnMode_unknown;
  last_known_azimuth_ = 0;
  pktcounter_ = 0;
}

HDLSensorType c_hdl_packet_parser::sensor_type() const
{
  return lidar_specification_.sensor;
}

HDLReturnMode c_hdl_packet_parser::return_mode() const
{
  return return_mode_;
}

const c_hdl_lidar_specifcation * c_hdl_packet_parser::lidar_specification() const
{
  return &lidar_specification_;
}

void c_hdl_packet_parser::set_hdl_framing_mode(enum HDLFramingMode v)
{
  hdl_framing_mode_ = v;
}

enum HDLFramingMode c_hdl_packet_parser::hdl_framing_mode() const
{
  return hdl_framing_mode_;
}

int c_hdl_packet_parser::last_known_azimuth() const
{
  return last_known_azimuth_;
}

int c_hdl_packet_parser::pktcounter() const
{
  return pktcounter_;
}

bool c_hdl_packet_parser::setup(HDLSensorType sensor_type, HDLReturnMode return_mode)
{
  this->return_mode_ = return_mode;

  bool use_default_lidar_specification = false;

  const std::string lidar_config_file =
      get_hdl_lidar_specification_config_file(sensor_type);

  if ( lidar_config_file.empty() ) {
    use_default_lidar_specification = true;
  }
  else {

    bool fOk =
        load_hdl_lidar_specifcation_db_xml(lidar_config_file,
            &lidar_specification_,
            sensor_type);

    if ( !fOk ) {
      CF_ERROR("load_hdl_lidar_specifcation_db_xml() fails for sensor_type=%s (%d) from xml file %s\n"
          "Will try to use default values for lidar calibrartion",
          toString(sensor_type), (int)sensor_type, lidar_config_file.c_str());
      use_default_lidar_specification = true;
    }
  }

  if ( use_default_lidar_specification ) {

    const c_hdl_lidar_specifcation * default_specification =
        get_default_hdl_lidar_specification(sensor_type);

    if ( !default_specification ) {
      CF_ERROR("get_default_hdl_lidar_specification(sensor_type=%s (%d)) fails",
          toString(sensor_type), (int )sensor_type);
      return false;
    }

    lidar_specification_ =
        *default_specification;
  }

  if ( !precompute_correction_tables() ) {
    CF_ERROR("precompute_timing_offsets() fails");
    return false;
  }

  if( sensor_type == HDLSensor_VLS128 ) {
    for( uint8_t i = 0; i < 16; ++i ) {
      vls_128_laser_azimuth_cache[i] =
          (VLS128_CHANNEL_TDURATION / VLS128_SEQ_TDURATION) * (i + i / 8);
    }
  }

  return true;
}

bool c_hdl_packet_parser::parse(const uint8_t * data, uint size)
{
  if( size != hdl_lidar_packet_size() ) {
    CF_ERROR("IGNORE PACKET: invalid size = %u", size);
    return true;
  }

  ++pktcounter_;

  const HDLDataPacket *dataPacket =
      reinterpret_cast<const HDLDataPacket*>(data);


  const HDLSensorType sensor_type =
      get_sensor_type(*dataPacket);

  const HDLReturnMode return_mode =
      get_return_mode(*dataPacket);

  if( lidar_specification_.sensor == HDLSensor_unknown ) {

    CF_DEBUG("SETUP %s %s", toString(sensor_type), toString(return_mode));

    if( !setup(sensor_type, return_mode) ) {
      CF_ERROR("setup() fails");
      return false;
    }
  }
  else if( sensor_type != lidar_specification_.sensor ) {
    CF_ERROR("Unexpected sensor type change: %s -> %s",
        toString(lidar_specification_.sensor), toString(sensor_type));
    return false;
  }
  else if( return_mode != this->return_mode_ ) {
    CF_ERROR("Unexpected return mode change: %s -> %s",
        toString(this->return_mode_), toString(return_mode));
    return false;
  }


  switch (sensor_type) {

  case HDLSensor_VLP16:
    if( !parse_vlp16(dataPacket) ) {
      return false;
    }
    break;

  case HDLSensor_VLP16HiRes:
    break;

  case HDLSensor_VLP32AB:
    case HDLSensor_VLP32C:
    if( !parse_vlp32(dataPacket) ) {
      return false;
    }
    break;

  case HDLSensor_HDL32E:
    if( !parse_hdl32(dataPacket) ) {
      return false;
    }
    break;

  case HDLSensor_HDL64:
    if( !parse_hdl64(dataPacket) ) {
      return false;
    }
    break;

  case HDLSensor_VLS128:
    if( !parse_vls128(dataPacket) ) {
      return false;
    }
    break;

  default:
    CF_ERROR("Unknown or not supported sensor type 0x%0X", sensor_type);
    return false;
  }

  return true;
}



/**
 * cached values precomputation, from Velodyne user manuals
 * */
bool c_hdl_packet_parser::precompute_correction_tables()
{
  precomputed_timing_offsets_.clear();
  precomuted_corrections_table_.clear();

  CF_DEBUG("sensor_type=%s return_mode=%s",
      toString(sensor_type()),
      toString(return_mode()));

  switch (sensor_type()) {

  case HDLSensor_VLP16: {
    /*
     * Copied from vtkVelodyneLegacyPacketInterpreter.cxx
     * */
    static const auto VLP16AdjustTimeStamp =
        [](int firingblock, int channelNumber, int firingwithinblock, bool isDualReturnMode) -> double {
          return isDualReturnMode ?
            (firingblock / 2 * 110.592) + (channelNumber * 2.304) + (firingwithinblock * 55.296) :
            (firingblock * 110.592) + (channelNumber * 2.304) + (firingwithinblock * 55.296);
        };

    const bool dual_mode =
        !is_single_return_mode(return_mode_);

    precomputed_timing_offsets_.resize(
        HDL_DATA_BLOCKS_PER_PKT + 2);

    for( int block = 0; block < HDL_DATA_BLOCKS_PER_PKT + 2; ++block ) {

      precomputed_timing_offsets_[block].resize(
          HDL_LASERS_PER_DATA_BLOCK);

      for( int K = 0; K < HDL_LASERS_PER_DATA_BLOCK; ++K ) {

        const int channelNumber = K < 16 ?
            K :
            K - 16;

        const int firingWithinBlock = K < 16 ?
            0 :
            1;

        precomputed_timing_offsets_[block][K] =
            VLP16AdjustTimeStamp(block,
                channelNumber,
                firingWithinBlock,
                dual_mode);
      }
    }

    break;
  }

  case HDLSensor_VLP32AB:
    case HDLSensor_VLP32C: {

    /*
     * Copied from vtkVelodyneLegacyPacketInterpreter.cxx
     * */
    static const auto VLP32AdjustTimeStamp =
        [](int firingblock, int dsr,
            bool isDualReturnMode) -> double {
              return isDualReturnMode ?
                  (firingblock / 2 * 55.296) + (dsr / 2) * 2.304 :
                  (firingblock * 55.296) + (dsr / 2) * 2.304;
            };

    const bool dual_mode =
        !is_single_return_mode(return_mode_);

    precomputed_timing_offsets_.resize(
        HDL_DATA_BLOCKS_PER_PKT + 2);

    for( int block = 0; block < HDL_DATA_BLOCKS_PER_PKT + 2; ++block ) {

      precomputed_timing_offsets_[block].resize(
          HDL_LASERS_PER_DATA_BLOCK);

      for( int K = 0; K < HDL_LASERS_PER_DATA_BLOCK; ++K ) {

        precomputed_timing_offsets_[block][K] =
            VLP32AdjustTimeStamp(block, K, dual_mode);
      }
    }

    break;
  }

  case HDLSensor_HDL32E: {
    /*
     * Copied from vtkVelodyneLegacyPacketInterpreter.cxx
     * */
    static const auto HDL32AdjustTimeStamp =
        [](int firingblock, int dsr, const bool isDualReturnMode) -> double {
          return isDualReturnMode ?
              (firingblock / 2 * 46.08) + (dsr * 1.152) :
              (firingblock * 46.08) + (dsr * 1.152);
        };

    const bool dual_mode =
        !is_single_return_mode(return_mode_);

    precomputed_timing_offsets_.resize(
        HDL_DATA_BLOCKS_PER_PKT + 2);

    for( int block = 0; block < HDL_DATA_BLOCKS_PER_PKT + 2; ++block ) {

      precomputed_timing_offsets_[block].resize(
          HDL_LASERS_PER_DATA_BLOCK);

      for( int K = 0; K < HDL_LASERS_PER_DATA_BLOCK; ++K ) {

        precomputed_timing_offsets_[block][K] =
            HDL32AdjustTimeStamp(block, K, dual_mode);
      }
    }

    break;
  }

  case HDLSensor_HDL64: {

    /*
     * Copied from vtkVelodyneLegacyPacketInterpreter.cxx
     * */
    static const auto HDL64EAdjustTimeStamp =
        [](int firingDataBlockIdx, int dsrBase32, bool isDualReturnMode) -> double
            {
              const int dsrBase32Reversed = HDL_LASERS_PER_DATA_BLOCK - dsrBase32 - 1;
              const int firingDataBlockReversed = HDL_DATA_BLOCKS_PER_PKT - firingDataBlockIdx - 1;

              if (!isDualReturnMode) {
                const double TimeOffsetMicroSec[4] = {2.34, 3.54, 4.74, 6.0};
                return (std::floor(static_cast<double>(firingDataBlockReversed) / 2.0) * 48.0) +
                    TimeOffsetMicroSec[(dsrBase32Reversed % 4)] + (dsrBase32Reversed / 4) * TimeOffsetMicroSec[3];
              }
              else {
                const double TimeOffsetMicroSec[4] = {3.5, 4.7, 5.9, 7.2};
                return (std::floor(static_cast<double>(firingDataBlockReversed) / 4.0) * 57.6) +
                    TimeOffsetMicroSec[(dsrBase32Reversed % 4)] + (dsrBase32Reversed / 4) * TimeOffsetMicroSec[3];
              }
            };


    const bool dual_mode =
        !is_single_return_mode(return_mode_);

    precomputed_timing_offsets_.resize(
        HDL_DATA_BLOCKS_PER_PKT + 4);

    for( int block = 0; block < HDL_DATA_BLOCKS_PER_PKT + 4; ++block ) {

      precomputed_timing_offsets_[block].resize(
          HDL_LASERS_PER_DATA_BLOCK);

      for( int K = 0; K < HDL_LASERS_PER_DATA_BLOCK; ++K ) {

        precomputed_timing_offsets_[block][K] =
            HDL64EAdjustTimeStamp(block, K, dual_mode);
      }
    }



    precomuted_corrections_table_.resize(lidar_specification_.lasers.size());
    for ( uint i = 0, n = lidar_specification_.lasers.size(); i < n; ++i ) {

      const c_hdl_lasers_table & laser =
          lidar_specification_.lasers[i];

      c_lasers_corrections_table & corrections =
          precomuted_corrections_table_[i];

      corrections.sin_rot_correction = sin(laser.rot_correction * M_PI / 180);
      corrections.cos_rot_correction = cos(laser.rot_correction * M_PI / 180);
      corrections.sin_vert_correction = sin(laser.vert_correction * M_PI / 180);
      corrections.cos_vert_correction = cos(laser.vert_correction * M_PI / 180);
    }

    return true;
  }

  case HDLSensor_VLS128: {

    constexpr uint nsequences = 3;
    constexpr uint nfiringGroups = 16 + 1; // 17 (+1 for the maintenance time after firing group 8)

    constexpr double full_firing_cycle = VLS128_SEQ_TDURATION * 1e-6; //seconds
    constexpr double single_firing = VLS128_CHANNEL_TDURATION * 1e-6; // seconds
    constexpr double offset_paket_time = VLS128_TOH_ADJUSTMENT * 1e-6; //seconds

    int sequence, firingGroup;

    precomputed_timing_offsets_.resize(nsequences);

    for( sequence = 0; sequence < nsequences; ++sequence ) {
      precomputed_timing_offsets_[sequence].resize(nfiringGroups);
    }

    for( sequence = 0; sequence < nsequences; ++sequence ) {
      for( firingGroup = 0; firingGroup < nfiringGroups; ++firingGroup ) {
        precomputed_timing_offsets_[sequence][firingGroup] =
            (full_firing_cycle * sequence) +
                (single_firing * firingGroup) -
                offset_paket_time;
      }
    }

    break;
  }

  case HDLSensor_VLP16HiRes: {
    return false;
  }

  default:
    return false;
  }

  return true;
}


bool c_hdl_packet_parser::parse_vlp16(const HDLDataPacket *dataPacket)
{
  if( lidar_specification_.lasers.size() != 16 ) {
    CF_ERROR("Invalid call: lasers_table was not correctly initialized for sensor '%s'. "
        "lidar_specification.lasers.size=%zu",
        toString(lidar_specification_.sensor),
        lidar_specification_.lasers.size());
    return false;
  }

  const c_hdl_lidar_specifcation &spec =
      lidar_specification_;

  const std::vector<c_hdl_lasers_table> &lasers_table =
      spec.lasers;

  const double packet_timestamp =
      1e-6 * dataPacket->TohTimestamp; // [s]

  const bool dual_mode =
      !is_single_return_mode(return_mode_);

  int azimuth_gap = 0;

  if( hdl_framing_mode_ == HDLFraming_Packet ) {
    frames.emplace_back(std::make_shared<c_lidar_frame>());
  }

  for( int block = 0; block < HDL_DATA_BLOCKS_PER_PKT; ++block ) {

    if( hdl_framing_mode_ == HDLFraming_DataBlock ) {
      frames.emplace_back(std::make_shared<c_lidar_frame>());
    }

    const HDLDataBlock &current_block =
        dataPacket->dataBlocks[block];

    const int current_azimuth =
        current_block.azimuth;

    if( !dual_mode ) {
      if( block < HDL_DATA_BLOCKS_PER_PKT - 1 ) {
        azimuth_gap = (int) dataPacket->dataBlocks[block + 1].azimuth - current_azimuth;
        if( azimuth_gap < 0 ) {
          azimuth_gap += 36000;
        }
      }
    }
    else {
      if( block < HDL_DATA_BLOCKS_PER_PKT - 2 ) {
        azimuth_gap = (int) dataPacket->dataBlocks[block + 2].azimuth - current_azimuth;
        if( azimuth_gap < 0 ) {
          azimuth_gap += 36000;
        }
      }
    }

    if( frames.empty() || (hdl_framing_mode_ == HDLFraming_Rotation && current_azimuth < last_known_azimuth_) ) {
      frames.emplace_back(std::make_shared<c_lidar_frame>());
    }

    last_known_azimuth_ =
        current_azimuth;

    const c_lidar_frame::sptr &current_frame =
        frames.back();

    const double blockdsr0 =
        precomputed_timing_offsets_[block][0];

    const double nextblockdsr0 =
        precomputed_timing_offsets_[block + (dual_mode ? 2 : 1)][0];

    for( int K = 0; K < HDL_LASERS_PER_DATA_BLOCK; ++K ) {

      const HDLLaserReturn &laserReturn =
          current_block.laserReturns[K];

      const int channelNumber =
          K < 16 ? K : K - 16;

      const int firingWithinBlock =
          K < 16 ? 0 : 1;


      const int laser_index =
          channelNumber;

      const c_hdl_lasers_table & laser =
          lasers_table[laser_index];

      const double distance =
          laserReturn.distance > 0 ?
              laserReturn.distance * spec.distance_resolution + laser.distance_correction :
              0;

      const double timestamp_adjustment =
          precomputed_timing_offsets_[block][K];

      const double azimuth_adjustment =
          azimuth_gap * ((timestamp_adjustment - blockdsr0) / (nextblockdsr0 - blockdsr0));

      const double timestamp =
          packet_timestamp + 1e-6 * timestamp_adjustment;

      double azimuth =
          1e-2 * (current_azimuth + azimuth_adjustment) -
              lasers_table[laser_index].rot_correction;

      while (azimuth >= 360) {
        azimuth -= 360;
      }
      while (azimuth < 0) {
        azimuth += 360;
      }

      c_lidar_point p = {
          .laser_id = laser_index,
          .laser_ring = lasers_table[laser_index].laser_ring,
          .pkt = pktcounter_,
          .datablock = block,
          .flags = 0,
          .azimuth = (double) (azimuth * M_PI / 180),
          .elevation = (double) (laser.vert_correction * M_PI / 180),
          .distance = distance,
          .intensity = laserReturn.intensity / 255.,
          .timestamp = timestamp,
      };

      current_frame->points.emplace_back(p);
    }
  }

  return true;
}


/**
 * VLP32CManual.pdf
 *
 * 9.3.2 Data Packet Structure
 *  There are two formats for the data packet:
 *    Single Return Mode (either Strongest or Last)
 *    Dual Return Mode
 *
 * 9.5 Precision Azimuth Calculation
 *  Perform the interpolation using the timing firing.
 *  Note that since pairs of lasers fire at once, each pair shares the same azimuth.
 *  See Table 9-5.
 */

bool c_hdl_packet_parser::parse_vlp32(const HDLDataPacket * dataPacket)
{
  if( lidar_specification_.lasers.size() != 32 ) {
    CF_ERROR("Invalid call: lasers_table was not correctly initialized for sensor '%s'. "
        "lidar_specification.lasers.size=%zu",
        toString(lidar_specification_.sensor),
        lidar_specification_.lasers.size());
    return false;
  }

  const c_hdl_lidar_specifcation &spec =
      lidar_specification_;

  const std::vector<c_hdl_lasers_table> &lasers_table =
      spec.lasers;

  const double packet_timestamp =
      1e-6 * dataPacket->TohTimestamp; // [s]

  const bool dual_mode =
      !is_single_return_mode(return_mode_);

  int azimuth_gap = 0;

  if( hdl_framing_mode_ == HDLFraming_Packet ) {
    frames.emplace_back(std::make_shared<c_lidar_frame>());
  }

  for( int block = 0; block < HDL_DATA_BLOCKS_PER_PKT; ++block ) {

    if( hdl_framing_mode_ == HDLFraming_DataBlock ) {
      frames.emplace_back(std::make_shared<c_lidar_frame>());
    }

    const HDLDataBlock &current_block =
        dataPacket->dataBlocks[block];

    const int current_azimuth =
        current_block.azimuth;

    if( !dual_mode ) {
      if( block < HDL_DATA_BLOCKS_PER_PKT - 1 ) {
        azimuth_gap = (int) dataPacket->dataBlocks[block + 1].azimuth - current_azimuth;
        if( azimuth_gap < 0 ) {
          azimuth_gap += 36000;
        }
      }
    }
    else {
      if( block < HDL_DATA_BLOCKS_PER_PKT - 2 ) {
        azimuth_gap = (int) dataPacket->dataBlocks[block + 2].azimuth - current_azimuth;
        if( azimuth_gap < 0 ) {
          azimuth_gap += 36000;
        }
      }
    }

    if( frames.empty() || (hdl_framing_mode_ == HDLFraming_Rotation && current_azimuth < last_known_azimuth_) ) {
      frames.emplace_back(std::make_shared<c_lidar_frame>());
    }

    last_known_azimuth_ =
        current_azimuth;

    const c_lidar_frame::sptr &current_frame =
        frames.back();

    const double blockdsr0 =
        precomputed_timing_offsets_[block][0];

    const double nextblockdsr0 =
        precomputed_timing_offsets_[block + (dual_mode ?2 : 1)][0];

    for( int K = 0; K < HDL_LASERS_PER_DATA_BLOCK; ++K ) {

      const HDLLaserReturn &laserReturn =
          current_block.laserReturns[K];

      const int laser_index =
          K;

      const c_hdl_lasers_table & laser =
          lasers_table[laser_index];

      const double distance =
          laserReturn.distance > 0 ?
              laserReturn.distance * spec.distance_resolution + laser.distance_correction :
              0;

      const double timestamp_adjustment =
          precomputed_timing_offsets_[block][K];

      const double azimuth_adjustment =
          azimuth_gap * ((timestamp_adjustment - blockdsr0) / (nextblockdsr0 - blockdsr0));

      const double timestamp =
          packet_timestamp + 1e-6 * timestamp_adjustment;

      double azimuth =
          1e-2 * (current_azimuth + azimuth_adjustment) -
              lasers_table[laser_index].rot_correction;

      while (azimuth >= 360) {
        azimuth -= 360;
      }
      while (azimuth < 0) {
        azimuth += 360;
      }

      c_lidar_point p = {
          .laser_id = laser_index,
          .laser_ring = lasers_table[laser_index].laser_ring,
          .pkt = pktcounter_,
          .datablock = block,
          .flags = 0,
          .azimuth = (double) (azimuth * M_PI / 180),
          .elevation = (double) (laser.vert_correction * M_PI / 180),
          .distance = distance,
          .intensity = laserReturn.intensity / 255.,
          .timestamp = timestamp,
      };

      current_frame->points.emplace_back(p);
    }
  }

  return true;
}

bool c_hdl_packet_parser::parse_hdl32(const HDLDataPacket * dataPacket)
{
  if( lidar_specification_.lasers.size() != 32 ) {
    CF_ERROR("Invalid call: lasers_table was not correctly initialized for sensor '%s'. "
        "lidar_specification.lasers.size=%zu",
        toString(lidar_specification_.sensor),
        lidar_specification_.lasers.size());
    return false;
  }

  const c_hdl_lidar_specifcation &spec =
      lidar_specification_;

  const std::vector<c_hdl_lasers_table> &lasers_table =
      spec.lasers;

  const double packet_timestamp =
      1e-6 * dataPacket->TohTimestamp; // [s]

  const bool dual_mode =
      !is_single_return_mode(return_mode_);

  int azimuth_gap = 0;

  if( hdl_framing_mode_ == HDLFraming_Packet ) {
    frames.emplace_back(std::make_shared<c_lidar_frame>());
  }

  for( int block = 0; block < HDL_DATA_BLOCKS_PER_PKT; ++block ) {

    if( hdl_framing_mode_ == HDLFraming_DataBlock ) {
      frames.emplace_back(std::make_shared<c_lidar_frame>());
    }

    const HDLDataBlock &current_block =
        dataPacket->dataBlocks[block];

    const int current_azimuth =
        current_block.azimuth;

    if( !dual_mode ) {
      if( block < HDL_DATA_BLOCKS_PER_PKT - 1 ) {
        azimuth_gap = (int) dataPacket->dataBlocks[block + 1].azimuth - current_azimuth;
        if( azimuth_gap < 0 ) {
          azimuth_gap += 36000;
        }
      }
    }
    else {
      if( block < HDL_DATA_BLOCKS_PER_PKT - 2 ) {
        azimuth_gap = (int) dataPacket->dataBlocks[block + 2].azimuth - current_azimuth;
        if( azimuth_gap < 0 ) {
          azimuth_gap += 36000;
        }
      }
    }

    if( frames.empty() || (hdl_framing_mode_ == HDLFraming_Rotation && current_azimuth < last_known_azimuth_) ) {
      frames.emplace_back(std::make_shared<c_lidar_frame>());
    }

    last_known_azimuth_ =
        current_azimuth;

    const c_lidar_frame::sptr &current_frame =
        frames.back();


    const double blockdsr0 =
        precomputed_timing_offsets_[block][0];

    const double nextblockdsr0 =
        precomputed_timing_offsets_[block + (dual_mode ? 2 : 1)][0];

    for( int K = 0; K < HDL_LASERS_PER_DATA_BLOCK; ++K ) {

      const HDLLaserReturn &laserReturn =
          current_block.laserReturns[K];

      const int laser_index =
          K;

      const c_hdl_lasers_table & laser =
          lasers_table[laser_index];

      const double distance =
          laserReturn.distance > 0 ?
              laserReturn.distance * spec.distance_resolution + laser.distance_correction :
              0;

      const double timestamp_adjustment =
          precomputed_timing_offsets_[block][K];

      const double azimuth_adjustment =
          azimuth_gap * ((timestamp_adjustment - blockdsr0) / (nextblockdsr0 - blockdsr0));

      const double timestamp =
          packet_timestamp + 1e-6 * timestamp_adjustment;

      double azimuth =
          1e-2 * (current_azimuth + azimuth_adjustment) -
              lasers_table[laser_index].rot_correction;

      while (azimuth >= 360) {
        azimuth -= 360;
      }
      while (azimuth < 0) {
        azimuth += 360;
      }

      c_lidar_point p = {
          .laser_id = laser_index,
          .laser_ring = lasers_table[laser_index].laser_ring,
          .pkt = pktcounter_,
          .datablock = block,
          .flags = 0,
          .azimuth = azimuth * M_PI / 180,
          .elevation = laser.vert_correction * M_PI / 180,
          .distance = distance,
          .intensity = laserReturn.intensity / 255.,
          .timestamp = timestamp,
      };

      current_frame->points.emplace_back(p);
    }
  }

  return true;
}

// HDL-64E_S3.pdf
bool c_hdl_packet_parser::parse_hdl64(const HDLDataPacket * dataPacket)
{
  if( lidar_specification_.lasers.size() != 64 ) {
    CF_ERROR("Invalid call: lasers_table was not correctly initialized for sensor '%s'. "
        "lidar_specification.lasers.size=%zu",
        toString(lidar_specification_.sensor),
        lidar_specification_.lasers.size());
    return false;
  }

  const c_hdl_lidar_specifcation & spec =
      lidar_specification_;

  const std::vector<c_hdl_lasers_table> &lasers_table =
      spec.lasers;

  const double distance_resolution =
      spec.distance_resolution;

  const double packet_timestamp =
      1e-6 * dataPacket->TohTimestamp; // [s]

  const bool dual_mode =
      !is_single_return_mode(return_mode_);

  int azimuth_gap = 0;

  if( hdl_framing_mode_ == HDLFraming_Packet ) {
    frames.emplace_back(std::make_shared<c_lidar_frame>());
  }

  for( int block = 0; block < HDL_DATA_BLOCKS_PER_PKT; ++block ) {

    if( hdl_framing_mode_ == HDLFraming_DataBlock ) {
      frames.emplace_back(std::make_shared<c_lidar_frame>());
    }

    const HDLDataBlock &current_block =
        dataPacket->dataBlocks[block];

    const int current_azimuth =
        current_block.azimuth;

    if ( !dual_mode ) {
      if( block < HDL_DATA_BLOCKS_PER_PKT - 2 ) {
        azimuth_gap = (int)dataPacket->dataBlocks[block + 2].azimuth - current_azimuth;
        if( azimuth_gap < 0 ) {
          azimuth_gap += 36000;
        }
      }
    }
    else {
      if( block < HDL_DATA_BLOCKS_PER_PKT - 4 ) {
        azimuth_gap = (int)dataPacket->dataBlocks[block + 4].azimuth - current_azimuth;
        if( azimuth_gap < 0 ) {
          azimuth_gap += 36000;
        }
      }
    }

    if( frames.empty() || (hdl_framing_mode_ == HDLFraming_Rotation && current_azimuth < last_known_azimuth_) ) {
      frames.emplace_back(std::make_shared<c_lidar_frame>());
    }

    last_known_azimuth_ =
        current_azimuth;

    const c_lidar_frame::sptr &current_frame =
        frames.back();

    int bank_origin = 0;
    if( current_block.blockId == HDL_BLOCK_32_63 ) {
      bank_origin = 32;
    }

    const double blockdsr0 =
        precomputed_timing_offsets_[block][0];

    const double nextblockdsr0 =
        precomputed_timing_offsets_[block + (dual_mode ? 4 : 2)][0];

    for( int K = 0; K < HDL_LASERS_PER_DATA_BLOCK; ++K ) {

      const HDLLaserReturn &laserReturn =
          current_block.laserReturns[K];

      const int laser_index =
          K + bank_origin;

      const c_hdl_lasers_table & laser =
          lasers_table[laser_index];

      const c_lasers_corrections_table & corrections =
          precomuted_corrections_table_[laser_index];

      const double timestamp_adjustment =
          precomputed_timing_offsets_[block][K];

      const double azimuth_adjustment =
          azimuth_gap * ((timestamp_adjustment - blockdsr0) / (nextblockdsr0 - blockdsr0));

      const double timestamp =
          packet_timestamp + 1e-6 * timestamp_adjustment;


      double corrected_azimuth, corrected_elevation, corrected_distance, corrected_intensity;

      if( !laserReturn.distance || (!laser.horz_offset && !laser.vert_offset) ) {
        corrected_azimuth =
            (1e-2 * (current_azimuth + azimuth_adjustment) - lasers_table[laser_index].rot_correction) * M_PI / 180;

        corrected_elevation =
            (lasers_table[laser_index].vert_correction * M_PI / 180);

        corrected_distance =
            laserReturn.distance > 0 ?
                laserReturn.distance * distance_resolution + laser.distance_correction :
                0;

        while (corrected_azimuth >= 2 * M_PI) {
          corrected_azimuth -= 2 * M_PI;
        }
        while (corrected_azimuth < 0) {
          corrected_azimuth += 2 * M_PI;
        }

      }
      else {

        const double azimuth =
            (current_azimuth + azimuth_adjustment) * M_PI / 18000;

        //
        const double distance =
            laserReturn.distance > 0 ?
                laserReturn.distance * distance_resolution + laser.distance_correction :
                0;

        // sin(a-b) = sin(a)*cos(b) - cos(a)*sin(b)
        // cos(a-b) = cos(a)*cos(b) + sin(a)*sin(b)
        const double sin_rot_angle =
            sin(azimuth) * corrections.cos_rot_correction - cos(azimuth) * corrections.sin_rot_correction;

        const double cos_rot_angle =
            cos(azimuth) * corrections.cos_rot_correction + sin(azimuth) * corrections.sin_rot_correction;

        const double xy_distance =
            distance * corrections.cos_vert_correction - laser.vert_offset * corrections.sin_vert_correction;

        // Calculate temporal X and Y, use absolute values.
        const double xx =
            std::abs(xy_distance * sin_rot_angle - laser.horz_offset * cos_rot_angle);

        const double yy =
            std::abs(xy_distance * cos_rot_angle + laser.horz_offset * sin_rot_angle);

        const double distance_corr_x = laser.dist_correction_x ?
            (laser.distance_correction - laser.dist_correction_x) * (xx - 2.4) / (25.04 - 2.4) +
                laser.dist_correction_x - laser.distance_correction :
            0;

        const double distance_corr_y = laser.dist_correction_y ?
            (laser.distance_correction - laser.dist_correction_y) * (yy - 1.93) / (25.04 - 1.93) +
                laser.dist_correction_y - laser.distance_correction :
            0;

        const double distance_x =
            distance + distance_corr_x;

        const double distance_y =
            distance + distance_corr_y;

        const double x =
            sin_rot_angle * (distance_x * corrections.cos_vert_correction - laser.vert_offset * corrections.sin_vert_correction) -
                laser.horz_offset * cos_rot_angle;

        const double y =
            cos_rot_angle * (distance_y * corrections.cos_vert_correction - laser.vert_offset * corrections.sin_vert_correction) +
                laser.horz_offset * sin_rot_angle;

        const double z =
            distance_y * corrections.sin_vert_correction + laser.vert_offset * corrections.cos_vert_correction;

        corrected_azimuth =
            atan2(y, x) + M_PI;

        corrected_elevation =
            atan2(z, sqrt(x * x + y * y));

        corrected_distance =
            sqrt(x * x + y * y + z * z);
      }

      if ( !laser.focal_distance ) {
        corrected_intensity = laserReturn.intensity / 255.0;
      }
      else {
        const double focal_offset =
            256 * SQR(1.0 - laser.focal_distance / 131.0);

        const double inside_abs_value =
            std::abs(focal_offset - 256 * SQR(1.0 - laserReturn.distance / 65535.0));

        corrected_intensity = std::max(0.,
            (inside_abs_value > 0 ?
                laserReturn.intensity + laser.focal_slope * inside_abs_value :
                laserReturn.intensity + laser.close_slope * inside_abs_value) / 255.);

      }

      c_lidar_point p = {
          .laser_id = laser_index,
          .laser_ring = lasers_table[laser_index].laser_ring,
          .pkt = pktcounter_,
          .datablock = block,
          .flags = 0,
          .azimuth = corrected_azimuth,
          .elevation = corrected_elevation,
          .distance = corrected_distance,
          .intensity = corrected_intensity,
          .timestamp = timestamp,
      };

      current_frame->points.emplace_back(p);
    }
  }

  return true;
}




/** VLS-128 User Manual
 *  9.5 Precision Azimuth Calculation
 */
bool c_hdl_packet_parser::parse_vls128(const HDLDataPacket * dataPacket)
{
  if( lidar_specification_.lasers.size() != 128 ) {
    CF_ERROR("Invalid call: lasers_table was not correctly initialized for sensor '%s'. "
        "lidar_specification.lasers.size=%zu",
        toString(lidar_specification_.sensor),
        lidar_specification_.lasers.size());
    return false;
  }


  const c_hdl_lidar_specifcation & spec = this->lidar_specification_;
  const std::vector<c_hdl_lasers_table> &lasers_table = spec.lasers;

  const bool dual_mode = !is_single_return_mode(return_mode_);

  const double packet_timestamp = 1e-6 * dataPacket->TohTimestamp; // [s]

  int azimuth_gap = 0;

  if( hdl_framing_mode_ == HDLFraming_Packet ) {
    frames.emplace_back(std::make_shared<c_lidar_frame>());
  }

  for( int block = 0; block < HDL_DATA_BLOCKS_PER_PKT - (4 * dual_mode); ++block ) {

    if( hdl_framing_mode_ == HDLFraming_DataBlock ) {
      frames.emplace_back(std::make_shared<c_lidar_frame>());
    }

    const HDLDataBlock & current_block =
        dataPacket->dataBlocks[block];

    const int current_azimuth =
        current_block.azimuth;

    // warning: this azimuth_gap calculation is incorrect,
    // actually we need future minus current azimuth
    if ( dual_mode ) {
      azimuth_gap = current_azimuth - last_known_azimuth_;
    }
    else if( block < HDL_DATA_BLOCKS_PER_PKT - 4 ) {
      azimuth_gap = (int)dataPacket->dataBlocks[block + 4].azimuth - current_azimuth;
    }
    while( azimuth_gap < 0 ) {
      azimuth_gap += 36000;
    }

    // Detect which bank of 32 lasers is in this block
    int bank_origin = 0;
    switch (current_block.blockId) {
    case HDL_BLOCK_00_31:
      bank_origin = 0;
      break;
    case HDL_BLOCK_32_63:
      bank_origin = 32;
      break;
    case HDL_BLOCK_64_95:
      bank_origin = 64;
      break;
    case HDL_BLOCK_96_127:
      bank_origin = 96;
      break;
    default:
      CF_ERROR("Invalid data block id 0x%0X in vls128 packet", current_block.blockId);
      return false;
    }

    if( frames.empty() || (hdl_framing_mode_ == HDLFraming_Rotation && current_azimuth < last_known_azimuth_) ) {
      frames.emplace_back(std::make_shared<c_lidar_frame>());
    }

    const c_lidar_frame::sptr &current_frame =
        frames.back();

    last_known_azimuth_ =
        current_azimuth;


    for( int K = 0; K < HDL_LASERS_PER_DATA_BLOCK; ++K ) {

      // Offset the laser in this block by which block it's in
      const int laser_index = K + bank_origin;

      // VLS-128 fires 8 lasers at a time
      const int firing_order = laser_index / 8;

      const HDLLaserReturn & laserReturn =
          current_block.laserReturns[K];

      const c_hdl_lasers_table & laser =
          spec.lasers[laser_index];

      const double distance =
          laserReturn.distance > 0 ?
              laserReturn.distance * spec.distance_resolution + laser.distance_correction :
              0;

      const double timestamp =
          packet_timestamp + precomputed_timing_offsets_[block / 4][firing_order + laser_index / 64];

      // correct for the laser azimuthal pattern and  rotation as a function of timing during the firings
      double interpolated_azimuth =
          1e-2 * (current_azimuth + azimuth_gap * vls_128_laser_azimuth_cache[firing_order]) -
              lasers_table[laser_index].rot_correction;
      while ( interpolated_azimuth >= 360 ) {
        interpolated_azimuth -= 360;
      }
      while ( interpolated_azimuth < 0 ) {
        interpolated_azimuth += 360;
      }

      c_lidar_point p = {
          .laser_id = laser_index,
          .laser_ring = lasers_table[laser_index].laser_ring,
          .pkt = pktcounter_,
          .datablock = block,
          .flags = 0,
          .azimuth = (double) (interpolated_azimuth * M_PI / 180),
          .elevation = (double) (laser.vert_correction * M_PI / 180),
          .distance = distance,
          .intensity = (double) (laserReturn.intensity / 255.0),
          .timestamp = timestamp,
      };

      current_frame->points.emplace_back(p);
    }
  }

  return true;
}



//
//static double VLS128AdjustTimeStamp(int firingDataBlockIdx, int dsrBase32, const bool isDualReturnMode,
//    int extDataPacketType)
//{
//  constexpr static double dt = 2.665;
//  constexpr static double firingsequence_num_cycles = 20;
//  constexpr static double firingsequence_duration = (dt * firingsequence_num_cycles);
//  constexpr static int n_datablocks_per_firingsequence = 4;
//  constexpr static int n_simultaneous_firing = 8;
//
//  //dsr >= 64 needs an additional two cycles of delay to account for interleaved maintenance cycles
//  if( !isDualReturnMode ) {
//    //convert dsr from 0->31 to 0->127
//    int dsr = (dsrBase32 + 32 * (firingDataBlockIdx % n_datablocks_per_firingsequence));
//
//    return (firingsequence_duration * static_cast<int>(firingDataBlockIdx / n_datablocks_per_firingsequence)) +
//        (static_cast<int>(dsr / n_simultaneous_firing) + (static_cast<int>(dsr / 64) * 2)) * dt;
//  }
//
//  if( extDataPacketType > HDL_EXT_MODE_NONE ) {
//    //convert dsr from 0->31 to 0->127
//    int dsr = (dsrBase32 + 32 * static_cast<int>(firingDataBlockIdx / 3));
//
//    return (static_cast<int>(dsr / n_simultaneous_firing) + (static_cast<int>(dsr / 64) * 2)) * dt;
//  }
//
//  //convert dsr from 0->31 to 0->127
//  int dsr = (dsrBase32 + 32 * static_cast<int>(firingDataBlockIdx / 2));
//
//  return (static_cast<int>(dsr / n_simultaneous_firing) + (static_cast<int>(dsr / 64) * 2)) * dt;
//}
//


