#include "modules/localization/msf/local_tool/data_extraction/pcd_exporter.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/time.h>

namespace apollo {
namespace localization {
namespace msf {

PCDExporter::PCDExporter(const std::string &pcd_folder)
{
    _pcd_folder = pcd_folder;
    std::string stamp_file = _pcd_folder + "/pcd_timestamp.txt";

    if ((_stamp_file_handle = fopen(stamp_file.c_str(), "a")) == NULL) {
        std::cerr << "Cannot open stamp file!" << std::endl;
    }
}

PCDExporter::~PCDExporter(){
    if (_stamp_file_handle != NULL) {
        fclose(_stamp_file_handle);
    }
}

void PCDExporter::CompensatedPcdCallback(
                const rosbag::MessageInstance &msg_instance) {
    std::cout << "Compensated pcd callback." << std::endl;
    sensor_msgs::PointCloud2::ConstPtr msg =
                    msg_instance.instantiate<sensor_msgs::PointCloud2>();

    static unsigned int index = 1;

    std::stringstream ss_pcd;
    ss_pcd << _pcd_folder  << "/" << index << ".pcd";
    std::string pcd_filename = ss_pcd.str();

    WritePcdFile(pcd_filename, msg);
    fprintf(_stamp_file_handle, "%d %lf\n", index, msg->header.stamp.toSec());
    
    ++index;
}

void PCDExporter::WritePcdFile(const std::string &filename,
    const sensor_msgs::PointCloud2::ConstPtr &msg) {
    pcl::PCLPointCloud2 pcl_cloud;
    pcl_conversions::toPCL(*msg, pcl_cloud);
    pcl::PCDWriter writer;
    writer.writeBinaryCompressed(filename, pcl_cloud);
}

}
}
}