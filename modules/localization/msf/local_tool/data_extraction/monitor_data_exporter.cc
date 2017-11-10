#include "modules/localization/msf/local_tool/data_extraction/pcd_exporter.h"
#include "modules/localization/msf/local_tool/data_extraction/location_exporter.h"
#include "modules/localization/msf/local_tool/data_extraction/rosbag_reader.h"
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

using namespace apollo::localization::msf;

int main(int argc, char **argv) {
    boost::program_options::options_description boost_desc("Allowed options");
    boost_desc.add_options()
            ("help", "produce help message")
            ("bag_file", 
                    boost::program_options::value<std::string>(), 
             "provide the output folder")
            ("out_folder", 
                    boost::program_options::value<std::string>(), 
             "provide the output folder")
            ("cloud_topic", 
                    boost::program_options::value<std::string>()->
                    default_value("/apollo/sensor/velodyne64/compensator/PointCloud2"), 
             "provide point cloud2 topic")
            ("gnss_loc_topic", 
                    boost::program_options::value<std::string>()->
                    default_value(""), 
             "provide gnss localization topic") 
            ("lidar_loc_topic", 
                    boost::program_options::value<std::string>()->
                    default_value(""), 
             "provide lidar localization topic")
            ("fusion_loc_topic", 
                    boost::program_options::value<std::string>()->
                    default_value(""), 
             "provide fusion localization topic");  

    boost::program_options::variables_map boost_args;
    boost::program_options::store(boost::program_options::parse_command_line(argc, 
                argv, boost_desc), boost_args);
    boost::program_options::notify(boost_args);

    if (boost_args.count("help")|| !boost_args.count("bag_file") 
                || !boost_args.count("out_folder")) {
        std::cout << boost_desc << std::endl;
        return 0;
    }

    const std::string bag_file = boost_args["bag_file"].as<std::string>();
    const std::string pcd_folder = boost_args["out_folder"].as<std::string>()
                                  + "/pcd";

    const std::string cloud_topic =  
                        boost_args["cloud_topic"].as<std::string>();
    const std::string gnss_loc_topic =  
                        boost_args["gnss_loc_topic"].as<std::string>();
    const std::string lidar_loc_topic =  
                        boost_args["lidar_loc_topic"].as<std::string>();
    const std::string fusion_loc_topic =  
                        boost_args["fusion_loc_topic"].as<std::string>();

    PCDExporter::Ptr pcd_exporter(new PCDExporter(pcd_folder));
    LocationExporter::Ptr loc_exporter(new LocationExporter(pcd_folder));
    RosbagReader reader;
    reader.Subscribe(cloud_topic, 
                    (BaseExporter::OnRosmsgCallback)&PCDExporter::CompensatedPcdCallback, 
                    pcd_exporter);
    reader.Subscribe(gnss_loc_topic, 
                    (BaseExporter::OnRosmsgCallback)&LocationExporter::GnssLocCallback, 
                    loc_exporter);
    reader.Subscribe(lidar_loc_topic, 
                    (BaseExporter::OnRosmsgCallback)&LocationExporter::LidarLocCallback, 
                    loc_exporter);
    reader.Subscribe(fusion_loc_topic, 
                    (BaseExporter::OnRosmsgCallback)&LocationExporter::FusionLocCallback, 
                    loc_exporter);

    reader.Read(bag_file);

    return 0;
}