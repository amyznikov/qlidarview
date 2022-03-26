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

    constexpr uint nblocks = 12;
    constexpr uint nfirings = 32;

    constexpr double full_firing_cycle = 55.296 * 1e-6; // seconds
    constexpr double single_firing = 2.304 * 1e-6; // seconds

    const bool dual_mode = !is_single_return_mode(return_mode_);

    int dataBlockIndex, dataPointIndex;

    precomputed_timing_offsets_.resize(nblocks);
    for( uint block = 0; block < nblocks; ++block ) {
      precomputed_timing_offsets_[block].resize(nfirings);
    }

    for( uint block = 0; block < nblocks; ++block ) {
      for( uint firing = 0; firing < nfirings; ++firing ) {
        if( dual_mode ) {
          dataBlockIndex = (block - (block % 2)) + (firing / 16);
        }
        else {
          dataBlockIndex = (block * 2) + (firing / 16);
        }
        dataPointIndex = firing % 16;
        precomputed_timing_offsets_[block][firing] =
            (full_firing_cycle * dataBlockIndex) +
                (single_firing * dataPointIndex);
      }
    }

    break;
  }

  case HDLSensor_VLP32AB:
    case HDLSensor_VLP32C: {

    constexpr uint nblocks = 12;
    constexpr uint nfirings = 32;

    constexpr double full_firing_cycle = 55.296 * 1e-6; // seconds
    constexpr double single_firing = 2.304 * 1e-6; // seconds

    const bool dual_mode = !is_single_return_mode(return_mode_);

    int dataBlockIndex, dataPointIndex;

    precomputed_timing_offsets_.resize(nblocks);
    for( uint block = 0; block < nblocks; ++block ) {
      precomputed_timing_offsets_[block].resize(nfirings);
    }

    for( size_t block = 0; block < nblocks; ++block ) {
      for( size_t firing = 0; firing < nfirings; ++firing ) {
        if( dual_mode ) {
          dataBlockIndex = block / 2;
        }
        else {
          dataBlockIndex = block;
        }
        dataPointIndex = firing / 2;
        precomputed_timing_offsets_[block][firing] =
            (full_firing_cycle * dataBlockIndex) +
                (single_firing * dataPointIndex);
      }
    }

    break;
  }

  case HDLSensor_HDL32E: {

    constexpr uint nblocks = 12;
    constexpr uint nfirings = 32;

    constexpr double full_firing_cycle = 46.080 * 1e-6; // seconds
    constexpr double single_firing = 1.152 * 1e-6; // seconds

    const bool dual_mode = !is_single_return_mode(return_mode_);

    double dataBlockIndex, dataPointIndex;

    precomputed_timing_offsets_.resize(nblocks);
    for( uint block = 0; block < nblocks; ++block ) {
      precomputed_timing_offsets_[block].resize(nfirings);
    }

    for( size_t block = 0; block < nblocks; ++block ) {
      for( size_t firing = 0; firing < nfirings; ++firing ) {
        if( dual_mode ) {
          dataBlockIndex = block / 2;
        }
        else {
          dataBlockIndex = block;
        }
        dataPointIndex = firing / 2;
        precomputed_timing_offsets_[block][firing] =
            (full_firing_cycle * dataBlockIndex) +
                (single_firing * dataPointIndex);
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

    precomputed_timing_offsets_.resize(HDL_DATA_BLOCKS_PER_PKT + 4);
    for( int block = 0; block < HDL_DATA_BLOCKS_PER_PKT + 4; ++block ) {

      precomputed_timing_offsets_[block].resize(HDL_LASERS_PER_DATA_BLOCK);

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
  /*
   * Copied from vtkVelodyneLegacyPacketInterpreter.cxx
   * */
  static const auto VLP16AdjustTimeStamp =
      [](int firingblock, int channelNumber, int firingwithinblock, bool isDualReturnMode) -> double {
        return isDualReturnMode ?
          (firingblock / 2 * 110.592) + (channelNumber * 2.304) + (firingwithinblock * 55.296) :
          (firingblock * 110.592) + (channelNumber * 2.304) + (firingwithinblock * 55.296);
      };

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
        VLP16AdjustTimeStamp(block, 0, 0, dual_mode);

    const double nextblockdsr0 =
        VLP16AdjustTimeStamp(block + (dual_mode ? 2 : 1), 0, 0, dual_mode);

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
          VLP16AdjustTimeStamp(block, channelNumber, firingWithinBlock, dual_mode);

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
  /*
   * Copied from vtkVelodyneLegacyPacketInterpreter.cxx
   * */
  static const auto VLP32AdjustTimeStamp =
      [](int firingblock, int dsr,
          bool isDualReturnMode) -> double
          {
            return isDualReturnMode ?
            (firingblock / 2 * 55.296) + (dsr / 2) * 2.304 :
            (firingblock * 55.296) + (dsr / 2) * 2.304;
          };

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
        VLP32AdjustTimeStamp(block, 0, dual_mode);

    const double nextblockdsr0 =
        VLP32AdjustTimeStamp(block + (dual_mode ?
            2 : 1), 0, dual_mode);

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
          K == 0 ?
              blockdsr0 :
              VLP32AdjustTimeStamp(block, K, dual_mode);

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
 * HDL32E.pdf
 *
 * 9.3.2 Data Packet Structure
 * 9.5 Precision Azimuth Calculation
 *
 */
bool c_hdl_packet_parser::parse_hdl32(const HDLDataPacket * dataPacket)
{
  /*
   * Copied from vtkVelodyneLegacyPacketInterpreter.cxx
   * */

  static const auto HDL32AdjustTimeStamp =
      [](int firingblock, int dsr, const bool isDualReturnMode) -> double  {
    return isDualReturnMode ? (firingblock / 2 * 46.08) + (dsr * 1.152) : (firingblock * 46.08) + (dsr * 1.152);
  };

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
        HDL32AdjustTimeStamp(block, 0, dual_mode);

    const double nextblockdsr0 =
        HDL32AdjustTimeStamp(block + (dual_mode ?
            2 : 1), 0, dual_mode);

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
          K == 0 ?
              blockdsr0 :
              HDL32AdjustTimeStamp(block, K, dual_mode);

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

    //    const int current_firing_sequence =
    //        dual_mode ? 0 :
    //            block / 4;

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
//
//static double VLP16AdjustTimeStamp(int firingblock, int channelNumber, int firingwithinblock,
//    const bool isDualReturnMode)
//{
//  return isDualReturnMode ?
//      (firingblock / 2 * 110.592) + (channelNumber * 2.304) + (firingwithinblock * 55.296) :
//      (firingblock * 110.592) + (channelNumber * 2.304) + (firingwithinblock * 55.296);
//}
//
//static double VLP32AdjustTimeStamp(int firingblock, int dsr, const bool isDualReturnMode)
//{
//  return isDualReturnMode ?
//      (firingblock / 2 * 55.296) + (dsr / 2) * 2.304 :
//      (firingblock * 55.296) + (dsr / 2) * 2.304;
//}
//
//static double HDL32AdjustTimeStamp(int firingblock, int dsr, const bool isDualReturnMode)
//{
//  return isDualReturnMode ?
//      (firingblock / 2 * 46.08) + (dsr * 1.152) :
//      (firingblock * 46.08) + (dsr * 1.152);
//}
//
//static double HDL64EAdjustTimeStamp(int firingDataBlockIdx, int dsrBase32, const bool isDualReturnMode)
//{
//  const int dsrBase32Reversed = HDL_LASER_PER_FIRING - dsrBase32 - 1;
//  const int firingDataBlockReversed = HDL_FIRING_PER_PKT - firingDataBlockIdx - 1;
//
//  if (!isDualReturnMode) {
//    static constexpr double TimeOffsetMicroSec[4] = { 2.34, 3.54, 4.74, 6.0 };
//    return (std::floor(static_cast<double>(firingDataBlockReversed) / 2.0) * 48.0) +
//      TimeOffsetMicroSec[(dsrBase32Reversed % 4)] + (dsrBase32Reversed / 4) * TimeOffsetMicroSec[3];
//  }
//  else {
//    static constexpr double TimeOffsetMicroSec[4] = { 3.5, 4.7, 5.9, 7.2 };
//    return (std::floor(static_cast<double>(firingDataBlockReversed) / 4.0) * 57.6) +
//      TimeOffsetMicroSec[(dsrBase32Reversed % 4)] + (dsrBase32Reversed / 4) * TimeOffsetMicroSec[3];
//  }
//}
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

//
//void c_hdl_packet_parser::process_firing(const HDL_FiringData * firingData,
//    int multiBlockFiringLaserIdOffset,
//    int firingDataBlockIdx,
//    int azimuthDiff, uint64_t timestamp,
//    uint rawtime,
//    bool isThisFiringDualReturnData,
//    bool isDualReturnPacket,
//    const HDL_FiringData * extData,
//    int extDataPacketType)
//{
//  // First return block of a dual return packet: init last point of laser
//  if( !isThisFiringDualReturnData && (!this->IsHDL64 || (this->IsHDL64 && ((firingDataBlockIdx % 4) == 0))) ) {
//    this->FirstPointIdOfDualReturnPair = 0; // this->Points->GetNumberOfPoints();
//  }
//
//
//  uint16_t firingElevation100th =
//      getElevation100th(*firingData);
//
//  CF_DEBUG("   firingDataBlockIdx=%d firingElevation100th=%u", firingDataBlockIdx, firingElevation100th);
//
//  for( int dsrBase32 = 0; dsrBase32 < HDL_LASER_PER_FIRING; ++dsrBase32 ) {
//
//    const uint8_t channelNumberOr_dsrBase32_forVLP16 =
//        static_cast<uint8_t>(dsrBase32 + multiBlockFiringLaserIdOffset);
//
//    uint8_t channelNumber = channelNumberOr_dsrBase32_forVLP16;
//    const uint16_t azimuth = getRotationalPosition(*firingData);
//
//    // Detect VLP-16 data and adjust laser id if necessary
//    int firingWithinBlock = 0;
//
//    if( this->CalibrationReportedNumLasers == 16 ) {
//
//      if( multiBlockFiringLaserIdOffset != 0 ) {
//        if( !this->alreadyWarnedForIgnoredHDL64FiringPacket ) {
//          CF_WARNING("Error: Received a HDL-64 UPPERBLOCK firing packet with \n"
//              "a VLP-16 calibration file. Ignoring the firing.");
//          this->alreadyWarnedForIgnoredHDL64FiringPacket = true;
//        }
//        return;
//      }
//
//      if( channelNumber >= 16 ) {
//        channelNumber -= 16;
//        firingWithinBlock = 1;
//      }
//    }
//
//
//    // Interpolate azimuths and timestamps per laser within firing blocks
//    double timestampadjustment = 0;
//    int azimuthadjustment = 0;
//
//    if( this->UseIntraFiringAdjustment ) {
//      double blockdsr0 = 0, nextblockdsr0 = 1;
//      switch (this->CalibrationReportedNumLasers) {
//      case 128:
//      {
//        timestampadjustment = VLS128AdjustTimeStamp(firingDataBlockIdx, dsrBase32, isDualReturnPacket,
//            extDataPacketType);
//
//        if( isDualReturnPacket ) {
//          // With VLS-128 dual return packets only one dsr0 per packet, so this method will be used to
//          // ensure azimuthadjustment is correctly derived below
//          if( extDataPacketType > HDL_EXT_MODE_NONE ) {
//            nextblockdsr0 = VLS128AdjustTimeStamp(11, 32, isDualReturnPacket, extDataPacketType);
//            blockdsr0 = VLS128AdjustTimeStamp(0, 0, isDualReturnPacket, extDataPacketType);
//          }
//          else {
//            nextblockdsr0 = VLS128AdjustTimeStamp(7, 32, isDualReturnPacket, extDataPacketType);
//            blockdsr0 = VLS128AdjustTimeStamp(0, 0, isDualReturnPacket, extDataPacketType);
//          }
//        }
//        else {
//          // dsr0 occurs every fourth block with VLS-128 single return packets
//          nextblockdsr0 = VLS128AdjustTimeStamp((firingDataBlockIdx / 4) * 4 + 4, 0, isDualReturnPacket,
//              extDataPacketType);
//          blockdsr0 = VLS128AdjustTimeStamp((firingDataBlockIdx / 4) * 4, 0, isDualReturnPacket, extDataPacketType);
//        }
//        break;
//      }
//      case 64:
//      {
//        timestampadjustment = -HDL64EAdjustTimeStamp(firingDataBlockIdx, dsrBase32, isDualReturnPacket);
//        nextblockdsr0 = -HDL64EAdjustTimeStamp(
//            firingDataBlockIdx + (isDualReturnPacket ?
//                4 : 2), 0, isDualReturnPacket);
//        blockdsr0 = -HDL64EAdjustTimeStamp(firingDataBlockIdx, 0, isDualReturnPacket);
//        break;
//      }
//      case 32:
//      {
//        if( this->ReportedSensor == HDLSensor_VLP32AB || this->ReportedSensor == HDLSensor_VLP32C ) {
//          timestampadjustment = VLP32AdjustTimeStamp(firingDataBlockIdx, dsrBase32, isDualReturnPacket);
//          nextblockdsr0 = VLP32AdjustTimeStamp(
//              firingDataBlockIdx + (isDualReturnPacket ?
//                  2 : 1), 0, isDualReturnPacket);
//          blockdsr0 = VLP32AdjustTimeStamp(firingDataBlockIdx, 0, isDualReturnPacket);
//        }
//        else {
//          timestampadjustment = HDL32AdjustTimeStamp(firingDataBlockIdx, dsrBase32, isDualReturnPacket);
//          nextblockdsr0 = HDL32AdjustTimeStamp(
//              firingDataBlockIdx + (isDualReturnPacket ?
//                  2 : 1), 0, isDualReturnPacket);
//          blockdsr0 = HDL32AdjustTimeStamp(firingDataBlockIdx, 0, isDualReturnPacket);
//        }
//        break;
//      }
//      case 16:
//      {
//        timestampadjustment =
//            VLP16AdjustTimeStamp(firingDataBlockIdx, channelNumber, firingWithinBlock, isDualReturnPacket);
//        nextblockdsr0 = VLP16AdjustTimeStamp(
//            firingDataBlockIdx + (isDualReturnPacket ?
//                2 : 1), 0, 0, isDualReturnPacket);
//        blockdsr0 = VLP16AdjustTimeStamp(firingDataBlockIdx, 0, 0, isDualReturnPacket);
//        break;
//      }
//
//      default:
//      {
//        timestampadjustment = 0.0;
//        blockdsr0 = 0.0;
//        nextblockdsr0 = 1.0;
//      }
//      }
//
//      azimuthadjustment =
//          std::round(azimuthDiff * ((timestampadjustment - blockdsr0) / (nextblockdsr0 - blockdsr0)));
//
//      timestampadjustment =
//          std::round(timestampadjustment);
//    }
//
//    if( (!this->IgnoreZeroDistances || firingData->laserReturns[dsrBase32].distance != 0.0) ) {
//
//      const uint16_t adjustedAzimuth =
//          (36000 + (static_cast<int>(azimuth) + azimuthadjustment)) % 36000;
//
//      //CF_DEBUG("   azimuth=%u adjustedAzimuth=%u", azimuth, adjustedAzimuth);
//
//      push_firing_data(channelNumber,
//          channelNumberOr_dsrBase32_forVLP16,
//          adjustedAzimuth,
//          firingElevation100th,
//          timestamp + static_cast<uint64_t>(timestampadjustment),
//          rawtime + static_cast<unsigned int>(timestampadjustment),
//          &(firingData->laserReturns[dsrBase32]),
//          isThisFiringDualReturnData,
//          extDataPacketType,
//          &(extData->laserReturns[dsrBase32]));
//    }
//  }
//}
//
//
//
//void c_hdl_packet_parser::push_firing_data(uint8_t channelNumber,
//    uint8_t channelNumberOr_dsrBase32_forVLP16,
//    uint16_t azimuth,
//    uint16_t firingElevation100th,
//    uint64_t timestamp,
//    uint rawtime,
//    const HDL_LaserReturn * laserReturn,
//    const bool isFiringDualReturnData,
//    const int extDataPacketType,
//    const HDL_LaserReturn * extData)
//{
//  azimuth %= 36000;
//  const uint thisPointId = 0;// this->Points->GetNumberOfPoints();
//  // double firingElevation = static_cast<double>(firingElevation100th) / 100.0;
//
//  // Compute raw position
//  bool applyIntensityCorrection =
//    this->WantIntensityCorrection && this->IsHDL64 && !(this->SensorPowerMode == HDL_CorrectionOn);
//
//  HDLRawValues rawValues(azimuth, firingElevation100th, laserReturn->distance, laserReturn->intensity);
//
//  CF_DEBUG("azimuth=%u firingElevation100th=%u distance=%u intensity=%u",
//      azimuth, firingElevation100th, laserReturn->distance, laserReturn->intensity);
//
//
//  HDLCorrectedValues correctedValues;
//
//  this->compute_corrected_values(
//      rawValues,
//      channelNumber,
//      correctedValues,
//      applyIntensityCorrection);
//
//  double & distanceM = correctedValues.distance;
//  short intensity = correctedValues.intensity;
//  double (& pos)[3] = correctedValues.position;
//
//  uint32_t temp = 0;
//
//  // Apply sensor transform
//  //  cout << "this->SensorTransform" << this->SensorTransform->GetPosition()[0]
//  // << " " << this->SensorTransform->GetPosition()[0] << " " << this->SensorTransform->GetPosition()[0] << endl;
//  //  if (SensorTransform) this->SensorTransform->InternalTransformPoint(pos, pos);
//  //  if (this->shouldBeCroppedOut(pos))
//  //    return;
//
//  if (extDataPacketType > HDL_EXT_MODE_NONE) {
//    //      const unsigned char * bytes = reinterpret_cast<const unsigned char*>(extData);
//    //      temp = (static_cast<unsigned int>(bytes[0]) << 16)
//    //           + (static_cast<unsigned int>(bytes[1]) << 8)
//    //           + (static_cast<unsigned int>(bytes[2]) << 0);
//    //
//    //      if (isFiringDualReturnData) {
//    //        if (this->HideDropPoints && ((temp & 0x800000) == 0x800000)) {
//    //          return;
//    //        }
//    //      }
//    //      else {
//    //        if (this->HideDropPoints && ((temp & 0x800) == 0x800)) {
//    //          return;
//    //        }
//    //      }
//  }
//
//  // Do not add any data before here as this might short-circuit
//  if( isFiringDualReturnData ) {
//
////    const uint dualPointId =
////        this->LastPointId[channelNumberOr_dsrBase32_forVLP16]; //Can't be channelNumber because of VLP16 dual mode data layout
////
////    if (dualPointId < this->FirstPointIdOfDualReturnPair) {
////      // No matching point from first set (skipped?)
////      InsertNextValueIfNotNull(this->Flags, DUAL_DOUBLED);
////      InsertNextValueIfNotNull(this->DistanceFlag, 0);
////      InsertNextValueIfNotNull(this->DualReturnMatching, -1); // std::numeric_limits<vtkIdType>::quiet_NaN()
////      InsertNextValueIfNotNull(this->IntensityFlag, 0);
////    }
////    else {
////      const short dualIntensity = this->Intensity->GetValue(dualPointId);
////      const double dualDistance = this->Distance->GetValue(dualPointId);
////      unsigned int firstFlags = this->Flags->GetValue(dualPointId);
////      unsigned int secondFlags = 0;
////
////      if( dualDistance == distanceM && intensity == dualIntensity ) {
////        if( this->IgnoreZeroDistances ) {
////          return; // ignore duplicate point and leave first with original flags
////        }
////        // Otherwise we add the duplicate point
////      }
////
////      if (dualIntensity < intensity) {
////        firstFlags &= ~HDL_DUAL_INTENSITY_HIGH;
////        secondFlags |= HDL_DUAL_INTENSITY_HIGH;
////      }
////      else {
////        firstFlags &= ~HDL_DUAL_INTENSITY_LOW;
////        secondFlags |= HDL_DUAL_INTENSITY_LOW;
////      }
////
////      if (dualDistance < distanceM) {
////        firstFlags &= ~HDL_DUAL_DISTANCE_FAR;
////        secondFlags |= HDL_DUAL_DISTANCE_FAR;
////      }
////      else {
////        firstFlags &= ~HDL_DUAL_DISTANCE_NEAR;
////        secondFlags |= HDL_DUAL_DISTANCE_NEAR;
////      }
////
////      // We will output only one point so return out of this
////      if (this->DualReturnFilter) {
////        if (!(secondFlags & this->DualReturnFilter)) {
////          // second return does not match filter; skip
////          SetValueIfNotNull(this->Flags, dualPointId, firstFlags);
////          SetValueIfNotNull(this->DistanceFlag, dualPointId, MapDistanceFlag(firstFlags));
////          SetValueIfNotNull(this->IntensityFlag, dualPointId, MapIntensityFlag(firstFlags));
////          return;
////        }
////        if (!(firstFlags & this->DualReturnFilter)) {
////          // first return does not match filter; replace with second return
////          this->Points->SetPoint(dualPointId, pos);
////          SetValueIfNotNull(this->Distance, dualPointId, distanceM);
////          SetValueIfNotNull(this->DistanceRaw, dualPointId, laserReturn->distance);
////          SetValueIfNotNull(this->Intensity, dualPointId, intensity);
////          SetValueIfNotNull(this->Timestamp, dualPointId, timestamp);
////          SetValueIfNotNull(this->RawTime, dualPointId, rawtime);
////          SetValueIfNotNull(this->Flags, dualPointId, secondFlags);
////          SetValueIfNotNull(this->DistanceFlag, dualPointId, MapDistanceFlag(secondFlags));
////          SetValueIfNotNull(this->IntensityFlag, dualPointId, MapIntensityFlag(secondFlags));
////          return;
////        }
////      }
////
////      SetValueIfNotNull(this->Flags, dualPointId, firstFlags);
////      SetValueIfNotNull(this->DistanceFlag, dualPointId, MapDistanceFlag(firstFlags));
////      SetValueIfNotNull(this->IntensityFlag, dualPointId, MapIntensityFlag(firstFlags));
////      InsertNextValueIfNotNull(this->Flags, secondFlags);
////      InsertNextValueIfNotNull(this->DistanceFlag, MapDistanceFlag(secondFlags));
////      InsertNextValueIfNotNull(this->IntensityFlag, MapIntensityFlag(secondFlags));
////      // The first return indicates the dual return
////      // and the dual return indicates the first return
////      InsertNextValueIfNotNull(this->DualReturnMatching, dualPointId);
////      SetValueIfNotNull(this->DualReturnMatching, dualPointId, thisPointId);
////    }
//  }
//  else {
////    InsertNextValueIfNotNull(this->Flags, DUAL_DOUBLED);
////    InsertNextValueIfNotNull(this->DistanceFlag, 0);
////    InsertNextValueIfNotNull(this->IntensityFlag, 0);
////    InsertNextValueIfNotNull(this->DualReturnMatching, -1); // std::numeric_limits<vtkIdType>::quiet_NaN()
//  }
//
////  this->Points->InsertNextPoint(pos);
////  InsertNextValueIfNotNull(this->PointsX, pos[0]);
////  InsertNextValueIfNotNull(this->PointsY, pos[1]);
////  InsertNextValueIfNotNull(this->PointsZ, pos[2]);
////  InsertNextValueIfNotNull(this->Azimuth, azimuth);
////  InsertNextValueIfNotNull(this->Intensity, intensity);
////  InsertNextValueIfNotNull(this->LaserId, channelNumber);
////  InsertNextValueIfNotNull(this->Timestamp, timestamp);
////  InsertNextValueIfNotNull(this->RawTime, rawtime);
////  InsertNextValueIfNotNull(this->Distance, distanceM);
////  InsertNextValueIfNotNull(this->DistanceRaw, laserReturn->distance);
////  this->LastPointId[channelNumberOr_dsrBase32_forVLP16] = thisPointId;
////  InsertNextValueIfNotNull(this->VerticalAngle, correctedValues.elevation);
////
////  if (extDataPacketType > HDL_EXT_MODE_NONE) {
////    if (isFiringDualReturnData) {
////      InsertNextValueIfNotNull(this->BinaryFlags, u32_to_str((temp & 0xFFF000) >> 12));
////      InsertNextValueIfNotNull(this->Drop, (temp & 0x800000) >> 23);
////      InsertNextValueIfNotNull(this->Confidence, (temp & 0x007000) >> 12);
////      InsertNextValueIfNotNull(this->Interference, (temp & 0x060000) >> 17);
////      InsertNextValueIfNotNull(this->SunLevel, (temp & 0x018000) >> 15);
////    }
////    else {
////      InsertNextValueIfNotNull(this->BinaryFlags, u32_to_str(temp & 0xFFF));
////      InsertNextValueIfNotNull(this->Drop, (temp & 0x800) >> 11);
////      InsertNextValueIfNotNull(this->Confidence, temp & 0x007);
////      InsertNextValueIfNotNull(this->Interference, (temp & 0x060) >> 5);
////      InsertNextValueIfNotNull(this->SunLevel, (temp & 0x018) >> 3);
////    }
////  }
//}
//
//
////-----------------------------------------------------------------------------
//void c_hdl_packet_parser::compute_corrected_values(
//    const HDLRawValues & rawValues,
//    const uint channelNumber,
//    HDLCorrectedValues & correctedValues,
//    bool correctIntensity) const
//{
//  const HDL_LaserCorrection * correction = &(this->laser_corrections_[channelNumber]);
//
//  correctedValues.intensity = rawValues.intensity;
//  correctedValues.elevation = static_cast<double>(rawValues.elevation) * 0.01 + correction->verticalCorrection;
//
//  double cosAzimuth, sinAzimuth;
//  if( correction->rotationalCorrection == 0 ) {
//    cosAzimuth = cos_lookup_table_[rawValues.azimuth];
//    sinAzimuth = sin_lookup_table_[rawValues.azimuth];
//  }
//  else {
//    // realAzimuth = rawValues.azimuth/100 - rotationalCorrection
//    // cos(a-b) = cos(a)*cos(b) + sin(a)*sin(b)
//    // sin(a-b) = sin(a)*cos(b) - cos(a)*sin(b)
//    cosAzimuth = cos_lookup_table_[rawValues.azimuth] * correction->cosRotationalCorrection +
//      sin_lookup_table_[rawValues.azimuth] * correction->sinRotationalCorrection;
//    sinAzimuth = sin_lookup_table_[rawValues.azimuth] * correction->cosRotationalCorrection -
//      cos_lookup_table_[rawValues.azimuth] * correction->sinRotationalCorrection;
//  }
//
//  double cosVertCorrection = correction->cosVertCorrection;
//  double sinVertCorrection = correction->sinVertCorrection;
//  double sinVertOffsetCorrection = correction->sinVertOffsetCorrection;
//
//  if( rawValues.elevation != 0 ) {
//    const double vertAngleRad = M_PI / 180.0 *
//        (correction->verticalCorrection + static_cast<double>(rawValues.elevation) / 100.0);
//    cosVertCorrection = std::cos(vertAngleRad);
//    sinVertCorrection = std::sin(vertAngleRad);
//    sinVertOffsetCorrection = correction->verticalOffsetCorrection * sinVertCorrection;
//  }
//
//  // Compute the distance in the xy plane (w/o accounting for rotation)
//  /**the new term of sinVertOffsetCorrection
//   * was added to the expression due to the mathemathical
//   * model we used.(c
//   */
//  double distanceMRaw = rawValues.distance * this->DistanceResolutionM;
//  double distanceM = distanceMRaw + correction->distanceCorrection;
//  double xyDistance = distanceM * cosVertCorrection - sinVertOffsetCorrection;
//
//  correctedValues.distance = distanceM;
//  correctedValues.position[0] = xyDistance * sinAzimuth - correction->horizontalOffsetCorrection * cosAzimuth;
//  correctedValues.position[1] = xyDistance * cosAzimuth + correction->horizontalOffsetCorrection * sinAzimuth;
//  correctedValues.position[2] = distanceM * sinVertCorrection + correction->verticalOffsetCorrection;
//
//  if( correctIntensity && (correction->minIntensity < correction->maxIntensity) ) {
//    // Compute corrected intensity
//
//    /* Please refer to the manual:
//     "Velodyne, Inc. ©2013  63‐HDL64ES3 REV G" Appendix F. Pages 45-46
//     PLease note: in the manual, focalDistance is in centimeters, distance is the raw short from
//     the laser
//     & the graph is in meter */
//
//    // Casting the input values to double for the computation
//    double computedIntensity = static_cast<double>(correctedValues.intensity);
//    double minIntensity = static_cast<double>(correction->minIntensity);
//    double maxIntensity = static_cast<double>(correction->maxIntensity);
//
//    // Rescale the intensity between 0 and 255
//    computedIntensity = (computedIntensity - minIntensity) / (maxIntensity - minIntensity) * 255.0;
//
//    if( computedIntensity < 0 ) {
//      computedIntensity = 0;
//    }
//
//    double focalOffset = 256 * pow(1.0 - correction->focalDistance / 131.0, 2);
//    double insideAbsValue = std::abs(
//        focalOffset - 256 * pow(1.0 - static_cast<double>(rawValues.distance) / 65535.0f, 2));
//
//    if( insideAbsValue > 0 ) {
//      computedIntensity = computedIntensity + correction->focalSlope * insideAbsValue;
//    }
//    else {
//      computedIntensity = computedIntensity + correction->closeSlope * insideAbsValue;
//    }
//    computedIntensity = std::max(std::min(computedIntensity, 255.0), 1.0);
//
//    correctedValues.intensity = static_cast<decltype(correctedValues.intensity)>(computedIntensity);
//  }
//}
