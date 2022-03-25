/*
 * c_lidar_frame.cc
 *
 *  Created on: Mar 17, 2022
 *      Author: amyznikov
 */

#include "c_lidar_frame.h"



cv::Vec3f convert_to_cartesian(const c_lidar_point & p)
{
  const double azimuth = p.azimuth;
  const double elevation = p.elevation;
  const double distance = p.distance;

  return cv::Vec3f(distance * cos(elevation) * sin(azimuth),
      distance * cos(elevation) * cos(azimuth),
      distance * sin(elevation));
}

void convert_to_cartesian(const c_lidar_frame * src, std::vector<cv::Vec3f> & positions)
{
  positions.clear();
  if ( src ) {
    for( const c_lidar_point &p : src->points ) {

      const double azimuth = p.azimuth;
      const double elevation = p.elevation;
      const double distance = p.distance;
      //  const double intensity = p.intensity;
      //  const double timestamp = p.tstamp * 1e-6;
      const double x = distance * cos(elevation) * sin(azimuth);
      const double y = distance * cos(elevation) * cos(azimuth);
      const double z = distance * sin(elevation);

      positions.emplace_back(x, y, z);
    }
  }
}

bool convert_to_cartesian(const c_lidar_frame * src, c_lidar_point_cloud * dst)
{
  dst->points.clear();
  dst->points.reserve(src->points.size());

  for( const c_lidar_point &p : src->points ) {

    const double azimuth = p.azimuth;
    const double elevation = p.elevation;
    const double distance = p.distance;
    const double intensity = p.intensity;
    const double timestamp = p.timestamp * 1e-6;
    const double x = distance * cos(elevation) * sin(azimuth);
    const double y = distance * cos(elevation) * cos(azimuth);
    const double z = distance * sin(elevation);

    dst->points.emplace_back(x, y, z, intensity, timestamp, 0);
  }

  return true;
}

c_lidar_point_cloud::sptr convert_to_cartesian(const c_lidar_frame::sptr & src)
{
  c_lidar_point_cloud::sptr dst =
      std::make_shared<c_lidar_point_cloud>();

  convert_to_cartesian(src.get(), dst.get());

  return dst;
}
