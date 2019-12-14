load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "offline_lidar_obstacle_perception",
    srcs = ["offline_lidar_obstacle_perception.cc"],
    deps = [
        "//cyber",
        "//modules/drivers/proto:sensor_proto",
        "//modules/map/proto:map_proto",
        "//modules/perception/base",
        "//modules/perception/common:perception_gflags",
        "//modules/perception/common/io:io_util",
        "//modules/perception/common/point_cloud_processing",
        "//modules/perception/common/sensor_manager",
        "//modules/perception/lidar/app",
        "//modules/perception/lidar/lib/classifier/fused_classifier",
        "//modules/perception/lidar/lib/classifier/fused_classifier:ccrf_type_fusion",
        "//modules/perception/lidar/lib/ground_detector/spatio_temporal_ground_detector",
        "//modules/perception/lidar/lib/object_builder",
        "//modules/perception/lidar/lib/object_filter_bank/roi_boundary_filter",
        "//modules/perception/lidar/lib/roi_filter/hdmap_roi_filter",
        "//modules/perception/lidar/lib/scene_manager/ground_service",
        "//modules/perception/lidar/lib/scene_manager/roi_service",
        "//modules/perception/lidar/lib/segmentation/cnnseg:cnn_segmentation",
        "//modules/perception/lidar/lib/tracker/multi_lidar_fusion:mlf_engine",
        "//modules/perception/lidar/lib/tracker/multi_lidar_fusion:mlf_track_object_matcher",
        "//modules/perception/lidar/lib/tracker/multi_lidar_fusion:mlf_tracker",
        "//modules/perception/proto:perception_config_schema_proto",
        "@eigen",
        "@pcl",
    ],
)

cpplint()
