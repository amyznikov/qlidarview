/*
 * c_lidar_frame_loader.h
 *
 *  Created on: Mar 14, 2022
 *      Author: amyznikov
 */

#ifndef __c_lidar_frame_loader_h__
#define __c_lidar_frame_loader_h__

#include "c_lidar_frame.h"
#include "c_hdl_lidar_specifcation.h"
#include <string>

class c_lidar_frame_loader
{
public:
  typedef c_lidar_frame_loader this_class;
  typedef std::unique_ptr<this_class> uptr;
  typedef std::shared_ptr<this_class> sptr;

  static uptr open_file(const std::string & filename, const std::string & options = "");


  virtual ~c_lidar_frame_loader() = default;

  virtual const std::string & filename() const = 0;

  virtual const std::string & options() const = 0;

  virtual const c_hdl_lidar_specifcation * lidar_specification() const = 0;
  virtual HDLReturnMode return_mode() const = 0;


  virtual void set_hdl_framing_mode(enum HDLFramingMode v) = 0;
  virtual enum HDLFramingMode hdl_framing_mode() const = 0;

  virtual c_lidar_frame::sptr load_next_frame() = 0;

};

#endif /* __c_lidar_frame_loader_h__ */
