/*
 * c_lidar_frame_loader.cc
 *
 *  Created on: Mar 14, 2022
 *      Author: amyznikov
 */

#include "c_lidar_frame_loader.h"
#include "c_hdl_pcap_file_loader.h"
#include <stdexcept>
#include <core/ssprintf.h>
#include <core/readdir.h>
#include <core/debug.h>


template<>
const c_enum_member* members_of<c_zlidar_id>()
{
  static constexpr c_enum_member members[] = {
      { c_zlidar_id_any, "any" },
      { c_zlidar_lidar_id_vlp16_1, "vlp16-1" },
      { c_zlidar_lidar_id_vlp16_2, "vlp16-2" },
      { c_zlidar_lidar_id_vls128, "vls128" },
      { c_zlidar_id_any, nullptr },
  };

  return members;
}


c_lidar_frame_loader::uptr c_lidar_frame_loader::open_file(const std::string & filename, const std::string & options)
{
  const std::string suffix =
      get_file_suffix(filename);

  if( strcasecmp(suffix.c_str(), ".pcap") == 0 || strcasecmp(suffix.c_str(), ".vpcap") == 0 ) {

    c_hdl_pcap_file_loader::uptr obj =
        std::make_unique<c_hdl_pcap_file_loader>();

    if( !obj->open(filename, options) ) {
      CF_ERROR("c_hdl_pcap_file_loader::open('%s') fails", filename.c_str());
      obj.reset();
    }

    return obj;
  }

  throw std::invalid_argument(ssprintf("File suffix '%s' is unknown or not yet supported", suffix.c_str()));
  return nullptr;
}
