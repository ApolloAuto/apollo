#include "modules/perception/obstacle/onboard/obstacle_perception.h"
#include "modules/common/log.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/common/time/time.h"
#include "modules/perception/lib/base/timer.h"

namespace apollo {
namespace perception {

ObstaclePerception::ObstaclePerception() : initialized_(false) {

}

ObstaclePerception::~ObstaclePerception() {

}

bool ObstaclePerception::Init() {
    // initialize lidar detector
    lidar_perception_.reset(new LidarProcess);
    if (lidar_perception_ == nullptr) {
        AERROR << "Failed to get LidarProcess instance";
        return false;
    }
    if (!lidar_perception_->Init()) {
        AERROR << "Failed to initialize lidar perception";
        return false;
    }
    // initialize radar detector
    radar_detector_.reset(BaseRadarDetectorRegisterer::GetInstanceByName(FLAGS_onboard_radar_detector));
    if (radar_detector_ == nullptr) {
        AERROR << "Failed to get RadarDetector plugin " << FLAGS_onboard_radar_detector;
        return false;
    }
    if (!radar_detector_->Init()) {
        AERROR << "Failed to initialize RadarDetector : " << FLAGS_onboard_radar_detector;
    }
    // initialize fusion
    fusion_.reset(BaseFusionRegisterer::GetInstanceByName(FLAGS_onboard_fusion));
    if (fusion_ == nullptr) {
        AERROR << "Failed to get fusion instance: " << FLAGS_onboard_fusion;
        return false;
    }
    if (!fusion_->Init()) {
        AERROR << "Failed to init fusion:" << FLAGS_onboard_fusion;
        return false;
    }
    // initialize visualizer
    if (FLAGS_enable_visualization) {
        frame_visualizer_.reset(new OpenglVisualizer());
        if (frame_visualizer_ == nullptr) {
            AERROR << "Failed to get OpenglVisualizer instance";
            return false;
        }
        if (!frame_visualizer_->Init()) {
        	AERROR << "Failed to init visualizer.";
        	return false;
        }
        initialized_ = true;
    }

    return true;
}

void ObstaclePerception::SetGlobalOffset(const Eigen::Vector3d& global_offset) {
    global_offset_ = global_offset;
}

bool ObstaclePerception::Process(SensorRawFrame* frame, std::vector<ObjectPtr>& out_objects) {
    if (frame == nullptr) {
        return false;
    }
    PERF_BLOCK_START();

    std::shared_ptr<SensorObjects> sensor_objects(new SensorObjects());
    if (frame->sensor_type_ == VELODYNE_64) {
        VelodyneRawFrame* velodyne_frame = dynamic_cast<VelodyneRawFrame*>(frame);
        if (lidar_perception_->Process(velodyne_frame->timestamp_,
                                       velodyne_frame->cloud_, 
                                       std::make_shared<Eigen::Matrix4d>(velodyne_frame->pose_))) {
            sensor_objects->objects = lidar_perception_->GetObjects();
            frame_content_.SetLidarPose(velodyne_frame->pose_);
	        frame_content_.SetLidarCloud(velodyne_frame->cloud_);
	        // _frame_content.SetLidarRoiCloud(roi_cloud);
            frame_content_.SetTrackedObjectsLidar(sensor_objects->objects);
        } else {
            AERROR << "Lidar perception error!, " 
                   << std::fixed << std::setprecision(12) << velodyne_frame->timestamp_;
            return false;
        }
        // _frame_content.update_timestamp(velodyne_frame->_timestamp);
        PERF_BLOCK_END("lidar_perception");
    } else if (frame->sensor_type_ == RADAR) {
        RadarRawFrame* radar_frame = dynamic_cast<RadarRawFrame*>(frame);
        RadarDetectorOptions options;
        options.radar2world_pose = &(radar_frame->pose_);
        options.car_linear_speed = radar_frame->car_linear_speed_;
        std::vector<ObjectPtr> objects;
        std::vector<PolygonDType> map_polygons;
        if (radar_detector_->Detect(radar_frame->raw_obstacles_, map_polygons, options, &objects)) {
            if (!objects.empty()) {
                sensor_objects->objects = objects;
                frame_content_.SetTrackedObjectsRadar(objects);
            } else {
                AERROR << "Radar detector result is nullptr!";
                return false;
            }
        } else {
            AERROR << "Radar perception error!, " 
                   << std::fixed << std::setprecision(12) << radar_frame->timestamp_;
            return false;
        }
        PERF_BLOCK_END("radar_detection");
    } else {
        AERROR << "Unknown sensor type : " << frame->sensor_type_;
        return false;
    }
    sensor_objects->sensor_type = frame->sensor_type_;
    sensor_objects->sensor_id = GetSensorType(RADAR);
    sensor_objects->timestamp = frame->timestamp_;
    sensor_objects->sensor2world_pose = frame->pose_;

    // fusion
    std::vector<SensorObjects> multi_sensor_objs;
    multi_sensor_objs.push_back(*sensor_objects);
    std::vector<ObjectPtr> fused_objects;
    if (!fusion_->Fuse(multi_sensor_objs, &fused_objects)) {
        AERROR << "Failed to fusion";
        return false;
    }
    PERF_BLOCK_END("sensor_fusion");

    out_objects = fused_objects;
    if (frame->sensor_type_ == VELODYNE_64) {
        frame_content_.SetTrackedObjectsFused(fused_objects);
    }

    if (FLAGS_enable_visualization && initialized_) {
        frame_visualizer_->Render(frame_content_);
    }

    return true;
}

} // namespace perception
} // namespace apollo