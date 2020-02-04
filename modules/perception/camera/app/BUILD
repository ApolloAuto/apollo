load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "camera_app_lib_proto",
    deps = [
        ":camera_app_lib",
    ],
)

proto_library(
    name = "camera_app_lib",
    srcs = ["perception.proto"],
)

cc_library(
    name = "debug_info",
    srcs = ["debug_info.cc"],
    hdrs = ["debug_info.h"],
    deps = [
        "//cyber",
        "//modules/perception/base",
        "//modules/perception/camera/common",
        "//modules/perception/camera/lib/interface",
    ],
)

cc_library(
    name = "obstacle_camera_perception",
    srcs = ["obstacle_camera_perception.cc"],
    hdrs = ["obstacle_camera_perception.h"],
    deps = [
        ":camera_app_lib_proto",
        ":debug_info",
        "//cyber",
        "//modules/perception/base",
        "//modules/perception/camera/common",
        "//modules/perception/camera/lib/calibration_service/online_calibration_service",
        "//modules/perception/camera/lib/calibrator/laneline:laneline_calibrator",
        "//modules/perception/camera/lib/feature_extractor/tfe:external_feature_extractor",
        "//modules/perception/camera/lib/feature_extractor/tfe:project_feature",
        "//modules/perception/camera/lib/feature_extractor/tfe:tracking_feat_extractor",
        "//modules/perception/camera/lib/interface",
        "//modules/perception/camera/lib/lane/detector/denseline:denseline_lane_detector",
        "//modules/perception/camera/lib/lane/postprocessor/denseline:denseline_lane_postprocessor",
        "//modules/perception/camera/lib/obstacle/detector/yolo:yolo_obstacle_detector",
        "//modules/perception/camera/lib/obstacle/postprocessor/location_refiner:location_refiner_obstacle_postprocessor",
        "//modules/perception/camera/lib/obstacle/tracker/omt:omt_obstacle_tracker",
        "//modules/perception/camera/lib/obstacle/transformer/multicue:multicue_obstacle_transformer",
        "//modules/perception/inference/utils:inference_cuda_util_lib",
        "//modules/perception/inference/utils:inference_util_lib",
        "//modules/perception/lib/utils",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
    ],
)

cc_library(
    name = "traffic_light_camera_perception",
    srcs = ["traffic_light_camera_perception.cc"],
    hdrs = ["traffic_light_camera_perception.h"],
    deps = [
        ":camera_app_lib_proto",
        "//cyber",
        "//modules/perception/camera/common",
        "//modules/perception/camera/lib/interface",
        "//modules/perception/camera/lib/traffic_light/detector/detection",
        "//modules/perception/camera/lib/traffic_light/detector/recognition",
        "//modules/perception/camera/lib/traffic_light/tracker:semantic_decision",
        "//modules/perception/lib/utils",
    ],
    alwayslink = True,
)

cc_library(
    name = "lane_camera_perception",
    srcs = ["lane_camera_perception.cc"],
    hdrs = ["lane_camera_perception.h"],
    deps = [
        ":camera_app_lib_proto",
        ":debug_info",
        "//cyber",
        "//modules/perception/base",
        "//modules/perception/camera/common",
        "//modules/perception/camera/lib/calibration_service/online_calibration_service",
        "//modules/perception/camera/lib/calibrator/laneline:laneline_calibrator",
        "//modules/perception/camera/lib/interface",
        "//modules/perception/camera/lib/lane/detector/darkSCNN:darkSCNN_lane_detector",
        "//modules/perception/camera/lib/lane/detector/denseline:denseline_lane_detector",
        "//modules/perception/camera/lib/lane/postprocessor/darkSCNN:darkSCNN_lane_postprocessor",
        "//modules/perception/camera/lib/lane/postprocessor/denseline:denseline_lane_postprocessor",
        "//modules/perception/inference/utils:inference_cuda_util_lib",
        "//modules/perception/inference/utils:inference_util_lib",
        "//modules/perception/lib/utils",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
    ],
)

cc_library(
    name = "cipv_camera",
    srcs = ["cipv_camera.cc"],
    hdrs = ["cipv_camera.h"],
    deps = [
        ":camera_app_lib_proto",
        "//cyber",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/configs/proto:vehicle_config_proto",
        "//modules/common/math:geometry",
        "//modules/common/proto:error_code_proto",
        "//modules/common/proto:geometry_proto",
        "//modules/common/proto:header_proto",
        "//modules/common/proto:pnc_point_proto",
        "//modules/perception/base",
        "//modules/perception/camera/common:lane_object",
        "//modules/perception/camera/lib/interface",
        "//modules/perception/lib/utils",
    ],
)

cpplint()
