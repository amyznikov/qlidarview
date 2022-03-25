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
    printf("%+.3f,", item.rotCorrection);
    printf("%+.3f,", item.vertCorrection);
    printf("%+.3f,", item.vertOffsetCorrection);
    printf("%+.3f,", item.horizOffsetCorrection);
    printf("%+.3f,", item.distCorrection);
    printf("%+.3f,", item.distCorrectionX);
    printf("%+.3f,", item.distCorrectionY);
    printf("%+.3f,", item.focalDistance);
    printf("%+.3f,", item.focalSlope);
    printf("%+.3f", item.closeSlope);
    printf("},\n");

  }

  return 0;
}


