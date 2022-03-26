#include <core/lidar/hdl_lidar_specifcation_db_xml.h>
#include <core/debug.h>


int main(int argc, char * argv[])
{

  std::string filename;
  c_hdl_lidar_specifcation spec;


  for( int i = 1; i < argc; ++i ) {
    if( strcasecmp(argv[i], "-help") == 0 || strcasecmp(argv[i], "--help") == 0 ) {
      printf("USAGE:\n"
          " dump_hdl_lidar_specifcation_xml <db.xml>\n"
          "");
      return 0;
    }

    if( filename.empty() ) {
      filename = argv[i];
      continue;
    }

    fprintf(stderr, "Invalid argument: %s\n", argv[i]);
    return 1;
  }


  if ( filename.empty() ) {
    fprintf(stderr, "No input file name specified\n");
    return 1;
  }


  cf_set_logfile(stderr);
  cf_set_loglevel(CF_LOG_DEBUG);


  if( !load_hdl_lidar_specifcation_db_xml(filename, &spec) ) {
    CF_ERROR("load_hdl_lidar_specifcation('%s') fails", filename.c_str());
    return 1;
  }

  for ( uint i = 0, n = spec.lasers.size(); i < n; ++i ) {

    const c_hdl_lasers_table & item = spec.lasers[i];

    printf("{");

    printf("%d,", item.laser_id);
    printf("%d,", item.laser_ring);
    printf("%+.3f,", item.rot_correction);
    printf("%+.3f,", item.vert_correction);
    printf("%+.3f,", item.vert_offset);
    printf("%+.3f,", item.horz_offset);
    printf("%+.3f,", item.distance_correction);
    printf("%+.3f,", item.dist_correction_x);
    printf("%+.3f,", item.dist_correction_y);
    printf("%+.3f,", item.focal_distance);
    printf("%+.3f,", item.focal_slope);
    printf("%+.3f", item.close_slope);
    printf("},\n");

  }

  return 0;
}


