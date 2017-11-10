#include "modules/localization/msf/local_tool/data_extraction/location_exporter.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/localization/proto/measure.pb.h"

namespace apollo {
namespace localization {
namespace msf {

LocationExporter::LocationExporter(const std::string &loc_file_folder)
{
    _gnss_loc_file = loc_file_folder + "gnss_loc.txt";
    _lidar_loc_file = loc_file_folder + "lidar_loc.txt";
    _fusion_loc_file = loc_file_folder + "fusion_loc.txt";

    if ((_gnss_loc_file_handle = fopen(_gnss_loc_file.c_str(), "a")) == NULL) {
        std::cerr << "Cannot open gnss localization file!" 
                    << std::endl;
    }

    if ((_lidar_loc_file_handle = fopen(_lidar_loc_file.c_str(), "a")) == NULL) {
        std::cerr << "Cannot open lidar localization file!" 
                    << std::endl;
    }

    if ((_fusion_loc_file_handle = fopen(_fusion_loc_file.c_str(), "a")) == NULL) {
        std::cerr << "Cannot open fusion localization file!" 
                    << std::endl;
    }
}

LocationExporter::~LocationExporter(){
    if (_gnss_loc_file_handle != NULL) {
        fclose(_gnss_loc_file_handle);
    }

    if (_lidar_loc_file_handle != NULL) {
        fclose(_lidar_loc_file_handle);
    }

    if (_fusion_loc_file_handle != NULL) {
        fclose(_fusion_loc_file_handle);
    }
}

void LocationExporter::GnssLocCallback(
                const rosbag::MessageInstance &msg_instance) {
    std::cout << "GNSS location callback." << std::endl;
    boost::shared_ptr<IntegMeasure> msg =
                    msg_instance.instantiate<IntegMeasure>();
    static unsigned int index = 1;
    
    double timestamp = msg->header().timestamp_sec();
    double x = msg->position().x();
    double y = msg->position().y(); 
    double z = msg->position().z();
    
    // double qx = msg->pose().orientation().qx();
    // double qy = msg->pose().orientation().qy();
    // double qz = msg->pose().orientation().qz();
    // double qw = msg->pose().orientation().qw();
    double qx = 0.0;
    double qy = 0.0;
    double qz = 0.0;
    double qw = 1.0;

    double std_x = std::sqrt(msg->measure_covar(0));
    double std_y = std::sqrt(msg->measure_covar(10));
    double std_z = std::sqrt(msg->measure_covar(20));

    fprintf(_gnss_loc_file_handle, "%d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n", 
        index, timestamp, x, y, z, qx, qy, qz,qw, std_x, std_y, std_z);

    ++index;
}

void LocationExporter::LidarLocCallback(
                const rosbag::MessageInstance &msg_instance) {
    std::cout << "Lidar location callback." << std::endl;
    boost::shared_ptr<IntegMeasure> msg =
                    msg_instance.instantiate<IntegMeasure>();
    static unsigned int index = 1;
    
    double timestamp = msg->header().timestamp_sec();
    double x = msg->position().x();
    double y = msg->position().y(); 
    double z = msg->position().z();
    
    // double qx = msg->pose().orientation().qx();
    // double qy = msg->pose().orientation().qy();
    // double qz = msg->pose().orientation().qz();
    // double qw = msg->pose().orientation().qw();
    double qx = 0.0;
    double qy = 0.0;
    double qz = 0.0;
    double qw = 1.0;

    double std_x = std::sqrt(msg->measure_covar(0));
    double std_y = std::sqrt(msg->measure_covar(10));
    double std_z = std::sqrt(msg->measure_covar(20));

    fprintf(_lidar_loc_file_handle, "%d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n", 
        index, timestamp, x, y, z, qx, qy, qz,qw, std_x, std_y, std_z);

    ++index;
}

void LocationExporter::FusionLocCallback(
                const rosbag::MessageInstance &msg_instance) {
    std::cout << "Fusion location callback." << std::endl;
    boost::shared_ptr<LocalizationEstimate> msg =
                    msg_instance.instantiate<LocalizationEstimate>();
    static unsigned int index = 1;
    
    double timestamp = msg->measurement_time();
    double x = msg->pose().position().x();
    double y = msg->pose().position().y(); 
    double z = msg->pose().position().z();
    
    double qx = msg->pose().orientation().qx();
    double qy = msg->pose().orientation().qy();
    double qz = msg->pose().orientation().qz();
    double qw = msg->pose().orientation().qw();

    double std_x = msg->uncertainty().position_std_dev().x();
    double std_y = msg->uncertainty().position_std_dev().y();
    double std_z = msg->uncertainty().position_std_dev().z();

    fprintf(_fusion_loc_file_handle, "%d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n", 
        index, timestamp, x, y, z, qx, qy, qz, qw, std_x, std_y, std_z);

    ++index;
}

}
}
}