load("//tools/install:install.bzl", "install", "install_files", "install_src_files")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "perception_testdata",
    srcs = glob([
        "testdata/**",
    ]),
)

install(
    name = "perception_testdata_install",
    data_dest = "perception/addition_data",
    data = [":perception_testdata"],
)

install(
    name = "install",
    data_dest = "perception",
    data = [
        ":cyberfile.xml",
        ":perception.BUILD",
    ],
    deps = [
        ":pb_perception",
        ":pb_hdrs",
        "//modules/perception/data:install",
        "//modules/perception/onboard/component:install",
        "//modules/perception/onboard/transform_wrapper:install",
        "//modules/perception/onboard/common_flags:install",
        "//modules/perception/production:install",
        "//modules/perception/pipeline:install",
        "//modules/perception/camera/lib/motion_service:install",
        "//modules/perception/camera/tools/offline:install",
        "//modules/perception/lidar/lib/tracker/multi_lidar_fusion:install",
        "//modules/perception/lidar/lib/tracker/semantic_map:install",
        ":perception_testdata_install"
    ],
)

install(
    name = "pb_hdrs",
    data_dest = "perception/include",
    data = [
        "//modules/perception/camera/app/proto:perception_cc_proto",
        "//modules/perception/camera/common/proto:object_template_meta_schema_cc_proto",
        "//modules/perception/camera/lib/obstacle/detector/smoke/proto:smoke_cc_proto",
        "//modules/perception/camera/proto:yolo_cc_proto",
        "//modules/perception/lidar/app/proto:lidar_obstacle_detection_config_cc_proto",
        "//modules/perception/lidar/app/proto:lidar_obstacle_tracking_config_cc_proto",
        "//modules/perception/lidar/lib/detector/cnn_segmentation/proto:cnnseg_param_cc_proto",
        "//modules/perception/lidar/lib/detector/cnn_segmentation/proto:spp_engine_config_cc_proto",
        "//modules/perception/lidar/lib/detector/ncut_segmentation/proto:ncut_param_cc_proto",
        "//modules/perception/lidar/lib/scene_manager/ground_service/proto:ground_service_config_cc_proto",
        "//modules/perception/lidar/lib/scene_manager/proto:scene_manager_config_cc_proto",
        "//modules/perception/lidar/lib/scene_manager/roi_service/proto:roi_service_cc_proto",
        "//modules/perception/onboard/proto:fusion_camera_detection_component_cc_proto",
        "//modules/perception/onboard/proto:fusion_component_config_cc_proto",
        "//modules/perception/onboard/proto:lidar_component_config_cc_proto",
        "//modules/perception/onboard/proto:motion_service_cc_proto",
        "//modules/perception/onboard/proto:radar_component_config_cc_proto",
        "//modules/perception/onboard/proto:trafficlights_perception_component_cc_proto",
        "//modules/perception/pipeline/proto/plugin:ccrf_type_fusion_config_cc_proto",
        "//modules/perception/pipeline/proto/plugin:dst_existence_fusion_config_cc_proto",
        "//modules/perception/pipeline/proto/plugin:dst_type_fusion_config_cc_proto",
        "//modules/perception/pipeline/proto/plugin:multi_lidar_fusion_config_cc_proto",
        "//modules/perception/pipeline/proto/plugin:pbf_gatekeeper_config_cc_proto",
        "//modules/perception/pipeline/proto/plugin:roi_boundary_filter_config_cc_proto",
        "//modules/perception/pipeline/proto/stage:cnnseg_config_cc_proto",
        "//modules/perception/pipeline/proto/stage:darkSCNN_cc_proto",
        "//modules/perception/pipeline/proto/stage:darkSCNN_postprocessor_cc_proto",
        "//modules/perception/pipeline/proto/stage:denseline_cc_proto",
        "//modules/perception/pipeline/proto/stage:denseline_postprocessor_cc_proto",
        "//modules/perception/pipeline/proto/stage:detection_cc_proto",
        "//modules/perception/pipeline/proto/stage:fused_classifier_config_cc_proto",
        "//modules/perception/pipeline/proto/stage:ground_service_detector_config_cc_proto",
        "//modules/perception/pipeline/proto/stage:hdmap_roi_filter_config_cc_proto",
        "//modules/perception/pipeline/proto/stage:location_refiner_cc_proto",
        "//modules/perception/pipeline/proto/stage:map_manager_config_cc_proto",
        "//modules/perception/pipeline/proto/stage:multicue_cc_proto",
        "//modules/perception/pipeline/proto/stage:ncut_config_cc_proto",
        "//modules/perception/pipeline/proto/stage:object_filter_bank_config_cc_proto",
        "//modules/perception/pipeline/proto/stage:omt_cc_proto",
        "//modules/perception/pipeline/proto/stage:pbf_tracker_config_cc_proto",
        "//modules/perception/pipeline/proto/stage:pointcloud_preprocessor_config_cc_proto",
        "//modules/perception/pipeline/proto/stage:probabilistic_fusion_config_cc_proto",
        "//modules/perception/pipeline/proto/stage:recognition_cc_proto",
        "//modules/perception/pipeline/proto/stage:semantic_cc_proto",
        "//modules/perception/pipeline/proto/stage:singlestage_cc_proto",
        "//modules/perception/pipeline/proto/stage:spatio_temporal_ground_detector_config_cc_proto",
        "//modules/perception/pipeline/proto/stage:tracking_feature_cc_proto",
        "//modules/perception/proto:motion_service_cc_proto",
        "//modules/perception/proto:perception_config_schema_cc_proto",
        "//modules/perception/proto:perception_ultrasonic_cc_proto",
        "//modules/perception/proto:rt_cc_proto",
        "//modules/perception/proto:sensor_meta_schema_cc_proto",
        "//modules/perception/proto:tracker_config_cc_proto",
    ],
)

install_files(
    name = "pb_perception",
    dest = "perception",
    files = [
        "//modules/perception/pipeline/proto/stage:fused_classifier_config_py_pb2",
        "//modules/perception/pipeline/proto/stage:map_manager_config_py_pb2",
        "//modules/perception/pipeline/proto/plugin:dst_existence_fusion_config_py_pb2",
        "//modules/perception/pipeline/proto/plugin:dst_type_fusion_config_py_pb2",
        "//modules/perception/proto:motion_service_py_pb2",
        "//modules/common_msgs/perception_msgs:perception_camera_py_pb2",
        "//modules/perception/proto:perception_config_schema_py_pb2",
        "//modules/common_msgs/perception_msgs:perception_lane_py_pb2",
        "//modules/common_msgs/perception_msgs:perception_obstacle_py_pb2",
        "//modules/perception/pipeline/proto/plugin:roi_boundary_filter_config_py_pb2",
        "//modules/perception/proto:perception_ultrasonic_py_pb2",
        "//modules/perception/proto:tracker_config_py_pb2",
        "//modules/common_msgs/perception_msgs:traffic_light_detection_py_pb2",
    ],
)

install_src_files(
    name = "install_src",
    deps = [
        ":install_perception_src",
        ":install_perception_hdrs"
    ],
)

install_src_files(
    name = "install_perception_src",
    src_dir = ["."],
    dest = "perception/src",
    filter = "*",
)

install_src_files(
    name = "install_perception_hdrs",
    src_dir = ["."],
    dest = "perception/include",
    filter = "*.h",
)
cpplint()
