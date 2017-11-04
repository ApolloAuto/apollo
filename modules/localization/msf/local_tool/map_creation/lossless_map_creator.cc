#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <vector>
#include "modules/localization/msf/local_map/lossless_map/lossless_map.h"
#include "modules/localization/msf/local_map/lossless_map/lossless_map_pool.h"
#include "modules/localization/msf/common/io/velodyne_utility.h"
#include "modules/localization/msf/common/util/extract_ground_plane.h"
#include "modules/localization/msf/common/util/system_utility.h"

namespace apollo {
namespace localization {
namespace msf {
typedef FeatureXYPlane::PointT PclPointT;
typedef FeatureXYPlane::PointCloudT PclPointCloudT;
typedef FeatureXYPlane::PointCloudPtrT PclPointCloudPtrT;

const unsigned int CAR_SENSOR_LASER_NUMBER = 64;

bool parse_command_line(int argc, char* argv[], 
                        boost::program_options::variables_map& vm) {
    boost::program_options::options_description desc("Allowd options");
    desc.add_options()
            ("help", "product help message")
            ("use_plane_inliers_only", boost::program_options::value<bool>()->required(), 
             "use plane inliers only")
            // ("use_plane_fitting_ransac", boost::program_options::value<bool>()->required(), 
            //  "use plane fitting ransac")
            ("pcd_folders", boost::program_options::value<std::vector<std::string> >()
                            ->multitoken()->composing()->required(),
             "pcd folders(repeated)")
            ("pose_files", boost::program_options::value<std::vector<std::string> >()
                            ->multitoken()->composing()->required(), 
             "pose files(repeated)")
            ("map_folder", boost::program_options::value<std::string>()->required(), 
             "map folder")
            ("zone_id", boost::program_options::value<int>()->required(), 
             "zone id")
            ("coordinate_type", boost::program_options::value<std::string>()->required(), 
             "coordinate type: UTM or LTM")
            ("map_resolution_type", boost::program_options::value<std::string>()->required(), 
             "map resolution type: single or multi")
            ("resolution", boost::program_options::value<float>()->default_value(0.125), 
             "optional: resolution for single resolution generation, default: 0.125");
    try {
        boost::program_options::store(
            boost::program_options::parse_command_line(argc, argv, desc), vm);
        if (vm.count("help")) {
            std::cerr << desc << std::endl;
            return false;
        }
        boost::program_options::notify(vm);
    } catch (std::exception& e) {
        std::cerr << "Error" << e.what() << std::endl;
        std::cerr << desc << std::endl;
        return false;
    } catch (...) {
        std::cerr << "Unknown error!" << std::endl;
        return false;
    }
    return true;
}

void variance_online(double& mean, double& var, unsigned int& N, double x) {
    ++N;
    double value = (x - mean) / N;
    double v1 = x - mean;
    mean += value;
    double v2 = x - mean;
    var = ((N - 1) * var + v1 * v2) / N;
}

} // msf
} // localization
} // apollo

using namespace apollo::localization::msf;

int main(int argc, char **argv) {
    FeatureXYPlane plane_extractor;

    boost::program_options::variables_map boost_args;
    if (!parse_command_line(argc, argv, boost_args)) {
        std::cerr << "Parse input command line failed." << std::endl;
        return -1;
    }

    const std::vector<std::string> pcd_folder_pathes = boost_args["pcd_folders"].as<std::vector<std::string> >();
    const std::vector<std::string> pose_files = boost_args["pose_files"].as<std::vector<std::string> >();
    if (pcd_folder_pathes.size() != pose_files.size()) {
        std::cerr << "The count of pcd folders is not equal pose files" << std::endl;
        return -1;
    }

    const std::string map_base_folder = boost_args["map_folder"].as<std::string>();
    bool use_plane_inliers_only = boost_args["use_plane_inliers_only"].as<bool>();
    // bool use_plane_fitting_ransac = boost_args["use_plane_fitting_ransac"].as<bool>();
    const int zone_id = boost_args["zone_id"].as<int>();
    const std::string coordinate_type = boost_args["coordinate_type"].as<std::string>();
    if (strcasecmp(coordinate_type.c_str(), "UTM") != 0 && 
        strcasecmp(coordinate_type.c_str(), "LTM") != 0) {
        std::cerr << "Coordinate type invalide. (UTM or LTM)" << std::endl;
        return -1;
    }
    const std::string map_resolution_type = boost_args["map_resolution_type"].as<std::string>();
    if (strcasecmp(map_resolution_type.c_str(), "single") != 0 &&
        strcasecmp(map_resolution_type.c_str(), "multi") != 0) {
        std::cerr << "Map resolution type invalide. (single or multi)" << std::endl;
        return -1;
    }
    
    float single_resolution_map = boost_args["resolution"].as<float>();
    if (fabs(single_resolution_map - 0.03125) > 1e-8 &&
        fabs(single_resolution_map - 0.0625) > 1e-8 &&
        fabs(single_resolution_map - 0.125) < 1e-8 &&
        fabs(single_resolution_map - 0.25) < 1e-8 &&
        fabs(single_resolution_map - 0.5) < 1e-8 &&
        fabs(single_resolution_map - 1.0) < 1e-8 &&
        fabs(single_resolution_map - 2.0) < 1e-8 &&
        fabs(single_resolution_map - 4.0) < 1e-8 &&
        fabs(single_resolution_map - 8.0) < 1e-8 &&
        fabs(single_resolution_map - 16.0) < 1e-8) {
        std::cerr << "Map resolution can only be: 0.03125, "
             << "0.0625, 0.125, 0.25, 0.5, 1.0, 2.0, "
             << "4.0, 8.0 or 16.0." << std::endl;
    }  

    const unsigned int num_trials = pcd_folder_pathes.size();
   
    // load all poses
    std::cerr << "Pcd folders are as follows:" << std::endl;
    for (std::size_t i = 0; i < num_trials; ++i) {
        std::cerr << pcd_folder_pathes[i] << std::endl;
    }
    std::vector<std::vector<Eigen::Affine3d> > ieout_poses(num_trials);
    std::vector<std::vector<double> > time_stamps(num_trials);
    std::vector<std::vector<unsigned int> > pcd_indices(num_trials);
    for (std::size_t i = 0; i < pose_files.size(); ++i) {
         velodyne::load_pcd_poses(pose_files[i], ieout_poses[i], 
                            time_stamps[i], pcd_indices[i]);
    }

    LosslessMapConfig conf;
    LosslessMap map(conf);
    LosslessMapConfig &loss_less_config 
            = static_cast<LosslessMapConfig&>(map.get_config());
    std::string map_folder_path = map_base_folder + "/map";
    if (!system::is_exists(map_folder_path)) {
        system::create_directory(map_folder_path);
    }
    map.set_map_folder_path(map_folder_path);
    for (size_t i = 0; i < pcd_folder_pathes.size(); ++i) {
        map.add_dataset(pcd_folder_pathes[i]);
    }
    if (strcasecmp(map_resolution_type.c_str(), "single") == 0) {
        loss_less_config.set_single_resolutions(single_resolution_map);
    } else {
        loss_less_config.set_multi_resolutions();
    }
    
    if (strcasecmp(coordinate_type.c_str(), "UTM") == 0) {
        loss_less_config._coordinate_type = "UTM";
    } else {
        loss_less_config._coordinate_type = "LTM";
        loss_less_config._map_range = 
            Rect2D<double>(-1638400.0, -1638400.0, 1638400.0, 1638400.0);
    }
     
    // Output Config file
    char file_buf[1024];
    snprintf(file_buf, 1024, "%s/map/config.xml", map_base_folder.c_str());
    loss_less_config.save(file_buf);
    
    snprintf(file_buf, 1024, "%s/map/config.txt", map_base_folder.c_str());
    FILE * file = fopen(file_buf, "a");

    if (file) {
        fprintf(file, "\n\nVeldoyne %dE\n", CAR_SENSOR_LASER_NUMBER);
        fprintf(file, "Map coordinate type: %s\n", loss_less_config._coordinate_type.c_str());
        // if (loss_less_config._coordinate_type == "LTM") {
        //     fprintf(file, "Map origin longitude: %lf\n", loss_less_config._origin_longitude);
        //     fprintf(file, "Map origin latitude: %lf\n", loss_less_config._origin_latitude);
        // }
        fprintf(file, "Map compression: %d\n", loss_less_config._map_is_compression);
        fprintf(file, "Map resolution: ");
        for (size_t i = 0; i < loss_less_config._map_resolutions.size(); ++i) {
            fprintf(file, "%lf, ", loss_less_config._map_resolutions[i]);
        }
        fprintf(file, "\nMap size: %lf %lf %lf %lf\n", loss_less_config._map_range.get_min_x(),
                loss_less_config._map_range.get_min_y(),
                loss_less_config._map_range.get_max_x(),
                loss_less_config._map_range.get_max_y());
        fprintf(file, "Map node size: %d x %d\n", 
                      loss_less_config._map_node_size_x, 
                      loss_less_config._map_node_size_y);
        fprintf(file, "Map row x col: \n");
        for (size_t i = 0; i < loss_less_config._map_resolutions.size(); ++i) {
            fprintf(file, "%u x %u, ",
                    MapNodeIndex::get_map_index_range_north(loss_less_config, i),
                    MapNodeIndex::get_map_index_range_east(loss_less_config, i));
        }
        fprintf(file, "Map image max intensity: %lf\n", loss_less_config._max_intensity_value);
        fprintf(file, "Map image max var: %lf\n", loss_less_config._max_intensity_var_value);
        fprintf(file, "PCD folders: \n");
        for (unsigned int trial = 0; trial < num_trials; ++trial) {
            fprintf(file, "%s\n", pcd_folder_pathes[trial].c_str());
        }
        fclose(file);
    }
    else {
        std::cerr << "Can't open file: " << "./map/config.txt" << std::endl;
    }

    LosslessMapNodePool lossless_map_node_pool(25, 8);
    lossless_map_node_pool.initial(&loss_less_config);
    map.init_thread_pool(1, 6);
    map.init_map_node_caches(12, 24);
    map.attach_map_node_pool(&lossless_map_node_pool);

    for (unsigned int trial = 0; trial < num_trials; ++trial) {
        for (unsigned int frame_idx = 0; frame_idx < ieout_poses[trial].size(); ++frame_idx) {
            unsigned int trial_frame_idx = frame_idx;
            const std::vector<Eigen::Affine3d >& poses = ieout_poses[trial];
            velodyne::VelodyneFrame velodyne_frame;
            std::string pcd_file_path;
            std::ostringstream ss; 
            ss << pcd_indices[trial][frame_idx];
            pcd_file_path = pcd_folder_pathes[trial] + "/" + ss.str() + ".pcd";
            const Eigen::Affine3d &pcd_pose = poses[trial_frame_idx];
            velodyne::load_pcds(pcd_file_path, trial_frame_idx,
                                pcd_pose, velodyne_frame, false);
            std::cout << "Loaded " << velodyne_frame.pt3ds.size() << "3D Points at Trial: "
                      << trial << " Frame: " << trial_frame_idx << "." << std::endl;

            for (size_t i = 0; i < velodyne_frame.pt3ds.size(); ++i) {
                Eigen::Vector3d& pt3d_local = velodyne_frame.pt3ds[i];
                unsigned char intensity = velodyne_frame.intensities[i];
                Eigen::Vector3d pt3d_global = velodyne_frame.pose * pt3d_local;
                map.set_value(pt3d_global, zone_id, intensity);
            }

            if (use_plane_inliers_only) {
                PclPointCloudPtrT pcl_pc = PclPointCloudPtrT(new PclPointCloudT);
                pcl_pc->resize(velodyne_frame.pt3ds.size());
                for (size_t i = 0; i < velodyne_frame.pt3ds.size(); ++i) {
                    PclPointT& pt = pcl_pc->at(i);
                    pt.x = velodyne_frame.pt3ds[i][0];
                    pt.y = velodyne_frame.pt3ds[i][1];
                    pt.z = velodyne_frame.pt3ds[i][2];
                    pt.intensity = static_cast<float>(velodyne_frame.intensities[i]);
                }

                plane_extractor.extract_xy_plane(pcl_pc);
                PclPointCloudPtrT& plane_pc = plane_extractor.get_xy_plane_cloud();

                for (unsigned int k = 0; k < plane_pc->size(); ++k) {
                    const PclPointT& plane_pt = plane_pc->at(k);
                    Eigen::Vector3d pt3d_local_double;
                    pt3d_local_double[0] = plane_pt.x;
                    pt3d_local_double[1] = plane_pt.y;
                    pt3d_local_double[2] = plane_pt.z;
                    unsigned char intensity = static_cast<unsigned char>(plane_pt.intensity);
                    Eigen::Vector3d pt3d_global = velodyne_frame.pose * pt3d_local_double;
                    map.set_value_layer(pt3d_global, zone_id, intensity);
                }
            }
        }
    }

    // Compute the ground height offset
    double mean_height_diff = 0;
    double var_height_diff = 0;
    unsigned int count_height_diff = 0;
    for (unsigned int trial = 0; trial < num_trials; ++trial) {
        for (unsigned int i = 0; i < ieout_poses[trial].size(); ++i) {
            const Eigen::Affine3d& ieout_pose = ieout_poses[trial][i];
            const Eigen::Vector3d& pt3d = ieout_pose.translation();
            unsigned int resolution_id = 0;
            if (use_plane_inliers_only) {
                // Use the altitudes from layer 0 (layer 1 internally in the Map).
                unsigned int layer_id = 0;
                std::vector<unsigned int> layer_counts;
                map.get_count_safe(pt3d, zone_id, resolution_id, layer_counts);
                if (layer_counts.size() == 0) {
                    std::cerr << "[FATAL ERROR] Map node should at least have one layer." 
                              << std::endl;
                }
                assert(layer_counts.size() > 0);
                if (layer_counts[layer_id] > 0) {
                    std::vector<float> layer_alts;
                    map.get_alt_safe(pt3d, zone_id, resolution_id, layer_alts);
                    if (layer_alts.size() == 0) {
                        std::cerr << "[FATAL ERROR] Map node should at least have one layer." 
                                  << std::endl;
                    }
                    assert(layer_alts.size() > 0);
                    float alt = layer_alts[layer_id];
                    double height_diff = pt3d[2] - alt;
                    variance_online(mean_height_diff,
                                                var_height_diff,
                                                count_height_diff,
                                                height_diff);
                }
            } else {
                // Use the altitudes from all layers
                unsigned int count = map.get_count_safe(pt3d, zone_id, resolution_id);
                if (count > 0) {
                    float alt = map.get_alt_safe(pt3d, zone_id, resolution_id);
                    double height_diff = pt3d[2] - alt;
                    variance_online(mean_height_diff,
                                                var_height_diff,
                                                count_height_diff,
                                                height_diff);
                }
            }
        }
    }

    map.get_config()._map_ground_height_offset = mean_height_diff;
    std::string config_path = map.get_config()._map_folder_path + "/config.xml";
    map.get_config().save(config_path);
    std::cout << "Mean: " << mean_height_diff << ", Var: " << var_height_diff << "." << std::endl;

    return 0;
}
