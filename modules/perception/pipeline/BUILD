load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    data_dest = "perception/addition_data/pipeline",
    data = [
        ":runtime_data",
    ],
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "config/*",
    ]),
)

cc_library(
  name = "data_frame",
  hdrs = [
    "data_frame.h",
  ],
  deps = [
    "//modules/perception/camera/common:camera_frame",
    "//modules/perception/fusion/base:fusion_frame",
    "//modules/perception/lidar/common:lidar_frame",
  ],
)

cc_library(
  name = "plugin_factory",
  srcs = [
    "plugin_factory.cc",
  ],
  hdrs = [
    "plugin_factory.h",
  ],
  deps = [
    ":plugin",
    "//modules/common/util:util_tool",
    "//modules/perception/fusion/lib/gatekeeper/pbf_gatekeeper",

    "//modules/perception/camera/lib/obstacle/camera_detection_postprocessor/camera_get_object:camera_get_object",
    "//modules/perception/camera/lib/obstacle/camera_detection_postprocessor/filter_bbox",
    "//modules/perception/camera/lib/obstacle/camera_detection_postprocessor/recover_bbox",
    "//modules/perception/camera/lib/obstacle/preprocessor/get_image_data",
    "//modules/perception/camera/lib/obstacle/preprocessor/resize_and_normalize",

    "//modules/perception/lidar/lib/classifier/fused_classifier:ccrf_type_fusion",
    "//modules/perception/lidar/lib/object_filter_bank/roi_boundary_filter",
    "//modules/perception/lidar/lib/pointcloud_detection_postprocessor/pointcloud_get_objects",
    "//modules/perception/lidar/lib/pointcloud_detection_preprocessor/pointcloud_down_sample",
    "//modules/perception/lidar/lib/tracker/multi_lidar_fusion:mlf_track_object_matcher",
    "//modules/perception/lidar/lib/tracker/multi_lidar_fusion:mlf_tracker",
    "//modules/perception/pipeline/proto:pipeline_config_cc_proto",
  ],
)

cc_library(
  name = "plugin",
  hdrs = [
    "plugin.h",
  ],
  deps = [
    "//modules/perception/pipeline/proto:pipeline_config_cc_proto",
  ],
)

cc_library(
  name = "stage",
  srcs = [
    "stage.cc",
  ],
  hdrs = [
    "stage.h",
  ],
  deps = [
    ":data_frame",
    ":plugin_factory",
    ":plugin",
    "//modules/perception/pipeline/proto:pipeline_config_cc_proto",
  ],
)

cc_library(
  name = "pipeline",
  srcs = [
    "pipeline.cc",
  ],
  hdrs = [
    "pipeline.h",
  ],
  deps = [
    ":stage",
    "//cyber",
    "//modules/common/util:util_tool",
    "//modules/perception/camera/lib/obstacle/camera_detection_postprocessor",
    "//modules/perception/camera/lib/obstacle/detector/smoke:smoke_obstacle_detector",
    "//modules/perception/camera/lib/obstacle/detector/yolo:yolo_obstacle_detector",
    "//modules/perception/camera/lib/obstacle/preprocessor:camera_detection_preprocessor",
    "//modules/perception/camera/lib/obstacle/tracker/omt:omt_obstacle_tracker",
    "//modules/perception/camera/lib/obstacle/tracker/omt2:omt_bev_tracker",
    "//modules/perception/camera/lib/obstacle/transformer/multicue:multicue_obstacle_transformer",
    "//modules/perception/camera/lib/traffic_light/detector/detection",
    "//modules/perception/camera/lib/traffic_light/detector/recognition",
    "//modules/perception/camera/lib/traffic_light/tracker:semantic_decision",
    "//modules/perception/fusion/lib/data_fusion/all_latest_fusion",
    "//modules/perception/fusion/lib/fusion_system/probabilistic_fusion",
    "//modules/perception/fusion/lib/gatekeeper:collect_fused_object",
    "//modules/perception/lidar/lib/classifier/fused_classifier",
    "//modules/perception/lidar/lib/detector/center_point_detection",
    "//modules/perception/lidar/lib/detector/mask_pillars_detection",
    "//modules/perception/lidar/lib/detector/point_pillars_detection",
    "//modules/perception/lidar/lib/detector/cnn_segmentation",
    "//modules/perception/lidar/lib/detector/ncut_segmentation",
    "//modules/perception/lidar/lib/map_manager",
    "//modules/perception/lidar/lib/object_builder",
    "//modules/perception/lidar/lib/object_filter_bank",
    "//modules/perception/lidar/lib/pointcloud_detection_postprocessor",
    "//modules/perception/lidar/lib/pointcloud_detection_preprocessor",
    "//modules/perception/lidar/lib/pointcloud_preprocessor",
    "//modules/perception/lidar/lib/tracker/multi_lidar_fusion:mlf_engine",
    "//modules/perception/pipeline/proto:pipeline_config_cc_proto",
    "//modules/perception/camera/lib/obstacle/postprocessor/location_refiner:location_refiner_obstacle_postprocessor",
    "//modules/perception/camera/lib/obstacle/detector/bev_detection:bev_obstacle_detector",
    "//modules/perception/camera/lib/obstacle/detector/caddn:caddn_obstacle_detector",
  ],
)

cpplint()
