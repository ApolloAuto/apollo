/******************************************************************************
 * Created on Fri Jan 11 2019
 *
 * Copyright (c) 2019 Baidu.com, Inc. All Rights Reserved
 *
 * @file: filename
 * @desc: description
 * @author: yuanyijun@baidu.com
  *****************************************************************************/

#include "cyber/common/time_conversion.h"
#include "pose_collection.h"
#include "pose_collection_agent.h"

namespace adu {
namespace workers {
namespace collection {


PoseCollectionAgent::PoseCollectionAgent(std::shared_ptr<JSonConf> sp_conf) {
    _sp_pj_transformer = std::make_shared<PJTransformer>(50); // TODO zone id
    _sp_conf = sp_conf;
    reset();
}

void PoseCollectionAgent::reset() {
    _sp_pose_collection = std::make_shared<PoseCollection>(_sp_conf);
}

void PoseCollectionAgent::on_bestgnsspos_callback(const std::shared_ptr<const apollo::drivers::gnss::GnssBestPose> &bestgnsspos)
{
    if ( _sp_pose_collection == nullptr ) {
        _sp_pose_collection = std::make_shared<PoseCollection>(_sp_conf);
    }
    
    // double gps_time = double(bestgnsspos->novatel_header().gps_week_milliseconds()) * 1000 + double( bestgnsspos->novatel_header().gps_week() ) * 604800000000; //7 * 24 * 3600 * 1e6;
    // double time_stamp = apollo::cyber::common::GpsToUnixMicroseconds(gps_time) / 1e6;
    double time_stamp = bestgnsspos->measurement_time(); // in seconds

    FramePose pose;
    if (_sp_conf->use_system_time) {
        pose.time_stamp = unixtime_now();
    } else {
        pose.time_stamp = time_stamp;
    }
    pose.latitude = bestgnsspos->latitude();
    pose.longitude = bestgnsspos->longitude();
    pose.altitude = bestgnsspos->height_msl(); //bestgnsspos->altitude();
    pose.solution_status = bestgnsspos->sol_status();
    pose.position_type = bestgnsspos->sol_type();
    pose.diff_age = bestgnsspos->differential_age();
    double latitude_std = bestgnsspos->latitude_std_dev();
    double longitude_std = bestgnsspos->longitude_std_dev();
    double altitude_std = bestgnsspos->height_std_dev();
    pose.local_std = std::sqrt(latitude_std * latitude_std + longitude_std * longitude_std + altitude_std * altitude_std);
    pose.tx = pose.longitude * DEGRESS_TO_RADIANS;
    pose.ty = pose.latitude * DEGRESS_TO_RADIANS;
    pose.tz = pose.altitude;
    _sp_pj_transformer->latlong_to_utm(1, 1, &pose.tx, &pose.ty, &pose.tz);

    std::lock_guard<std::mutex> mutex_locker(_mutex);
    static FILE * pose_file = fopen("poses.txt", "w");
    static int count = 0;
    fprintf(stderr, "%d:%lf %lf %lf %lf 0.0 0.0 0.0 0.0\n", ++count, pose.time_stamp, pose.tx, pose.ty, pose.tz);
    fprintf(pose_file, "%lf %lf %lf %lf 0.0 0.0 0.0 0.0\n", pose.time_stamp, pose.tx, pose.ty, pose.tz);
    fflush(pose_file);
    _sp_pose_collection->collect(pose);
}

std::shared_ptr<std::vector<FramePose>> PoseCollectionAgent::get_poses() {
    if ( _sp_pose_collection == nullptr ) {
        return nullptr;
    }
    return _sp_pose_collection->get_poses();
}

} // collection
} // workers
} // adu
