/*
 * c_lidar_frame.h
 *
 *  Created on: Mar 15, 2022
 *      Author: amyznikov
 */

#ifndef __c_lidar_frame_h__
#define __c_lidar_frame_h__


#include <opencv2/opencv.hpp>
#include <vector>
#include <memory>

/**
 * This enum will specify additional flags for each lidar point, like dual return etc.
 * At this moment is not yet implemented.
 */
enum c_lidar_point_flags {};


/**
 * single lidar point as received from packet.
 * */
struct c_lidar_point {
  int laser_id;
  int laser_ring;
  int pkt;
  int datablock;
  int flags;
  double azimuth;
  double elevation;
  double distance;
  double intensity;
  double timestamp;
};

/**
 * lidar frame is the list of lidar points collected during single lidar rotation
 * */
class c_lidar_frame {
public:
  typedef c_lidar_frame this_class;
  typedef std::shared_ptr<this_class> sptr;
  typedef std::unique_ptr<this_class> uptr;

  std::vector<c_lidar_point> points;
};


/**
 * lidar point cloud represented in Cartesian XYZ coordinates
 * */
class c_lidar_point_cloud
{
public:
  typedef c_lidar_point_cloud this_class;
  typedef std::shared_ptr<this_class> sptr;
  typedef std::unique_ptr<this_class> uptr;

  // x, y, z, r, t, flag
  std::vector<cv::Vec6f> points;
};

/**
 * convert spherical (raw) lidar points to spherical coordinates
 */

cv::Vec3f convert_to_cartesian(const c_lidar_point & p);

void convert_to_cartesian(const c_lidar_frame * src,
    std::vector<cv::Vec3f> & positions);

bool convert_to_cartesian(const c_lidar_frame * src,
    c_lidar_point_cloud * dst);

c_lidar_point_cloud::sptr convert_to_cartesian(
    const c_lidar_frame::sptr & src);

#endif /* __c_lidar_frame_h__ */
