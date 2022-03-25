/*
 * pcapr.cc
 *
 *  Created on: Mar 11, 2022
 *      Author: amyznikov
 *
 *  Utility to read and parse velodyne pcap files.
 *  Created mainly for testing and debugging purposes.
 */

#include <core/lidar/c_hdl_pcap_file_loader.h>
#include <opencv2/opencv.hpp>
#include <core/io/save_image.h>
#include <core/ssprintf.h>
#include <core/debug.h>


//#define HDL_Grabber_toRadians(x) ((x)*M_PI / 180.0)






int main(int argc, char *argv[])
{

  std::string input_file_name;

  //
  // Parse command line arguments
  //
  for ( int i = 1; i < argc; ++i ) {

    if ( strcasecmp(argv[i], "-help") == 0 || strcasecmp(argv[i], "--help") == 0 ) {

      fprintf(stdout, "Velodyne pcap file reader\n"
          "USAGE:\n"
          "  pcapr [OPTIONS] input-file-name.pcap\n"
          "\n"
          "OPTIONS:\n"
          "  None yet\n");

      return 0;
    }


    if ( input_file_name.empty() ) {
      input_file_name = argv[i];
      continue;
    }


    fprintf(stderr, "Invalid argument: %s\n", argv[i]);
    return 1;
  }


  if ( input_file_name.empty() ) {
    fprintf(stderr, "No input file name specified\n");
    return 1;
  }

  //
  // Go open and read
  //

  cf_set_logfile(stderr);
  cf_set_loglevel(CF_LOG_DEBUG);


  c_hdl_pcap_file_loader loader;

  if ( !loader.open(input_file_name, "udp") ) {
    CF_ERROR("pcap.open('%s') fails", input_file_name.c_str());
    return 1;
  }

  CF_DEBUG("loader.open() OK");

  c_lidar_frame::sptr frame;
  cv::Mat1f depths;
  cv::Mat1b counts;
  cv::Mat1b diodes;
  cv::Mat1w pkts;
  cv::Mat1b blocks;
  cv::Mat1b intensity;

  //static constexpr double azimuthal_resolution = 0.199 * CV_PI / 180; // radian/pix
  static constexpr double azimuthal_resolution = 0.19 * CV_PI / 180; // radian/pix
  //static constexpr double azimuthal_resolution = 0.1 * CV_PI / 180; // radian/pix
  static const cv::Size image_size(2 * CV_PI / azimuthal_resolution, 32);

  CF_DEBUG("image size=%dx%d", image_size.width, image_size.height);


  for( int i = 0; i < 1000; ++i ) {

    depths.create(image_size);
    depths.setTo(0);

    counts.create(image_size);
    counts.setTo(0);

    diodes.create(image_size);
    diodes.setTo(0);

    pkts.create(image_size);
    pkts.setTo(0);

    blocks.create(image_size);
    blocks.setTo(0);

    intensity.create(image_size);
    intensity.setTo(0);

    if( !(frame = loader.load_next_frame()) ) {
      break;
    }

    for( const c_lidar_point & p : frame->points ) {

      int r = image_size.height - p.laser_ring - 1;
      int c = cvRound (p.azimuth / azimuthal_resolution);

      if ( r >= 0 && r < image_size.height ) {
        if ( c >= 0 && c < image_size.width ) {
          depths[r][c] = p.distance;
          counts[r][c] ++;
          diodes[r][c] = p.laser_id + 1;
          pkts[r][c] = p.pkt;
          blocks[r][c] = p.datablock;
          intensity[r][c] = p.intensity;
        }
      }
    }

    save_image(depths, ssprintf("depths/depths.%05d.tiff", i));
    save_image(counts, ssprintf("counts/counts.%05d.tiff", i));
    save_image(diodes, ssprintf("diodes/diodes.%05d.tiff", i));
    save_image(pkts, ssprintf("pkts/pkts.%05d.tiff", i));
    save_image(blocks, ssprintf("blocks/blocks.%05d.tiff", i));
    save_image(intensity, ssprintf("intensity/intensity.%05d.tiff", i));
  }

  return 0;
}


