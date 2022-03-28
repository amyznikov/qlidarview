/*
 * c_velodyne_pcap_file_loader.cc
 *
 *  Created on: Mar 14, 2022
 *      Author: amyznikov
 */

#include "c_hdl_pcap_file_loader.h"
#include <core/debug.h>

c_hdl_pcap_file_loader::~c_hdl_pcap_file_loader()
{
  close();
}

bool c_hdl_pcap_file_loader::open(const std::string & filename, const std::string & options)
{
  hdl_parser_.reset();

  if( !pcap_.open(filename, options) ) {
    CF_ERROR("pcap_.open('%s') fals", filename.c_str());
    return false;
  }

  return true;
}

const std::string & c_hdl_pcap_file_loader::filename() const
{
  return pcap_.filename();
}

const std::string & c_hdl_pcap_file_loader::options() const
{
  return pcap_.options();
}

const c_hdl_lidar_specifcation * c_hdl_pcap_file_loader::lidar_specification() const
{
  return hdl_parser_.lidar_specification();
}

HDLReturnMode c_hdl_pcap_file_loader::return_mode() const
{
  return hdl_parser_.return_mode();
}

void c_hdl_pcap_file_loader::set_hdl_framing_mode(enum HDLFramingMode v)
{
  hdl_parser_.set_hdl_framing_mode(v);
}

enum HDLFramingMode c_hdl_pcap_file_loader::hdl_framing_mode() const
{
  return hdl_parser_.hdl_framing_mode();
}

void c_hdl_pcap_file_loader::set_hdl_frame_seam_azimuth(double azimuth_in_degrees)
{
  hdl_parser_.set_hdl_frame_seam_azimuth(azimuth_in_degrees);
}

double c_hdl_pcap_file_loader::hdl_frame_seam_azimuth() const
{
  return hdl_parser_.hdl_frame_seam_azimuth();
}

void c_hdl_pcap_file_loader::set_zlidarid(uint32_t v)
{
  zlidarid_ = v;
}

uint32_t c_hdl_pcap_file_loader::zlidarid() const
{
  return zlidarid_;
}

void c_hdl_pcap_file_loader::close()
{
  pcap_.close();
}

c_lidar_frame::sptr c_hdl_pcap_file_loader::load_next_frame()
{
  if ( !pcap_.is_open() ) {
    CF_ERROR("pcap file is not open");
    return nullptr;
  }

  static const auto pop_frame =
      [](c_hdl_packet_parser & hdl_parser) -> c_lidar_frame::sptr {
        c_lidar_frame::sptr frame = hdl_parser.frames.front();
        hdl_parser.frames.erase(hdl_parser.frames.begin());
        return frame;
      };


  const pcap_pkthdr * pkt_header = nullptr;
  const c_pcap_data_header * data_header =  nullptr;
  const uint8_t * payload = nullptr;

  c_lidar_frame::sptr frame;

  const HDLFramingMode framing_mode =
      hdl_parser_.hdl_framing_mode();

  int status;


  if( hdl_parser_.frames.size() > 0 ) {
    if( hdl_parser_.frames.size() > 1 ) {
      return pop_frame(hdl_parser_);
    }
    if( framing_mode != HDLFraming_Rotation ) {
      return pop_frame(hdl_parser_);
    }
  }


  while ((status = pcap_.read(&pkt_header, &data_header, &payload)) >= 0) {

    if ( !payload ) {
      continue;
    }

    const uint payload_size =
        pkt_header->len - pcap_.data_header_size();

    if ( pcap_.datalinktype() == DLT_USER1 && zlidarid_ != -1 ) {
      if ( data_header->user1.zlidarid != zlidarid_ ) {
        continue;
      }
    }

    if( !hdl_parser_.parse(payload, payload_size) ) {
      CF_ERROR("packet_parser_.parse() fails");
      break;
    }

    if( hdl_parser_.frames.size() > 1 ) {
      frame = pop_frame(hdl_parser_);
      break;
    }

  }

  if ( status <= 0 ) {
    CF_ERROR("pcap_.read() fails");
  }

  if( !frame && hdl_parser_.frames.size() > 0 ) {
    frame = pop_frame(hdl_parser_);
  }


  return frame;
}
