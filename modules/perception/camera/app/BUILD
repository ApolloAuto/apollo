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
    srcs = [
        "perception.proto",
    ],
)

cc_library(
    name = "debug_info",
    srcs = [
        "debug_info.cc",
    ],
    hdrs = [
        "debug_info.h",
    ],
    deps = [
        "//cyber",
        "//modules/perception/base",
        "//modules/perception/camera/common",
        "//modules/perception/camera/lib/interface",
    ],
)

cc_library(
    name = "obstacle_camera_perception",
    srcs = [
        "obstacle_camera_perception.cc",
    ],
    hdrs = [
        "obstacle_camera_perception.h",
    ],
    deps = [
        ":camera_app_lib_proto",
        ":debug_info",
        "//cyber",
        "//external:gflags",
        "//modules/common/util:file_util",
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
        "@yaml_cpp//:yaml",
    ],
)

cc_library(
    name = "traffic_light_camera_perception",
    srcs = [
        "traffic_light_camera_perception.cc",
    ],
    hdrs = [
        "traffic_light_camera_perception.h",
    ],
    deps = [
        ":camera_app_lib_proto",
        "//cyber",
        "//modules/common/util:file_util",
        "//modules/perception/camera/common",
        "//modules/perception/camera/lib/interface",
        "//modules/perception/camera/lib/traffic_light/detector/detection",
        "//modules/perception/camera/lib/traffic_light/detector/recognition",
        "//modules/perception/camera/lib/traffic_light/tracker:semantic_decision",
        "//modules/perception/lib/utils",
    ],
)

cpplint()
