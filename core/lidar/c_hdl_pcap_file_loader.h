/*
 * c_hdl_pcap_file_loader.h
 *
 *  Created on: Mar 14, 2022
 *      Author: amyznikov
 */

#ifndef __c_velodyne_pcap_file_loader_h__
#define __c_velodyne_pcap_file_loader_h__

#include <core/lidar/c_lidar_frame_loader.h>
#include <core/lidar/c_hdl_packet_parser.h>
#include <core/io/c_pcap_file.h>
#include <string>

class c_hdl_pcap_file_loader:
    public c_lidar_frame_loader
{
public:
  typedef c_hdl_pcap_file_loader this_class;
  typedef c_lidar_frame_loader base;
  typedef std::unique_ptr<this_class> uptr;
  typedef std::shared_ptr<this_class> sptr;

  ~c_hdl_pcap_file_loader();

  bool open(const std::string & filename, const std::string & options = "");
  void close();

  const std::string & filename() const override;
  const std::string & options() const override;
  const c_hdl_lidar_specifcation * lidar_specification() const override;
  HDLReturnMode return_mode() const override;

  void set_hdl_framing_mode(enum HDLFramingMode v) override;
  enum HDLFramingMode hdl_framing_mode() const override;

  void set_hdl_frame_seam_azimuth(double azimuth_in_degrees) override;
  double hdl_frame_seam_azimuth() const override;

  void set_zlidarid(uint32_t v) override;
  uint32_t zlidarid() const override;

  c_lidar_frame::sptr load_next_frame() override;

protected:
  c_hdl_packet_parser hdl_parser_;
  c_pcap_reader pcap_;
  uint32_t zlidarid_ = -1;
};

#endif /* __c_velodyne_pcap_file_loader_h__ */
