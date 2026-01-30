load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

PERCEPTION_COPTS = ['-DMODULE_NAME=\\"perception\\"']

apollo_cc_library(
    name = "apollo_perception_camera_detection_bev",
    srcs = [
        "detector/petr/bev_obstacle_detector.cc",
        "detector/petr/postprocess.cc",
        "detector/petr/preprocess.cc",
        "interface/base_obstacle_detector.cc",
    ],
    hdrs = [
        "camera_frame.h",
        "detector/petr/bev_obstacle_detector.h",
        "detector/petr/postprocess.h",
        "detector/petr/preprocess.h",
        "interface/base_obstacle_detector.h",
    ],
    deps = [
        "//cyber",
        "//modules/common_msgs/sensor_msgs:sensor_image_cc_proto",
        "//modules/perception/camera_detection_bev/detector/petr/proto:model_param_cc_proto",
        "//modules/perception/camera_detection_bev/proto:camera_detection_bev_cc_proto",
        "//modules/perception/common:perception_common_util",
        "//modules/perception/common/base:apollo_perception_common_base",
        "//modules/perception/common/camera:apollo_perception_common_camera",
        "//modules/perception/common/inference:apollo_perception_common_inference",
        "//modules/perception/common/lib:apollo_perception_common_lib",
        "//modules/perception/common/onboard:apollo_perception_common_onboard",
        "//modules/perception/common/proto:model_info_cc_proto",
        "//modules/perception/common/proto:model_process_cc_proto",
        "@com_google_protobuf//:protobuf",
        "@eigen",
        "@opencv//:imgproc",
    ],
)

apollo_cc_library(
    name = "camera_detection_bev_component",
    srcs = [
        "camera_detection_bev_component.cc",
    ],
    hdrs = [
        "camera_detection_bev_component.h",
    ],
    copts = PERCEPTION_COPTS,
    deps = [
        ":apollo_perception_camera_detection_bev",
        "//cyber",
        "//modules/common_msgs/sensor_msgs:sensor_image_cc_proto",
        "//modules/perception/camera_detection_bev/proto:camera_detection_bev_cc_proto",
        "//modules/perception/common:perception_common_util",
        "//modules/perception/common/base:apollo_perception_common_base",
        "//modules/perception/common/camera:apollo_perception_common_camera",
        "//modules/perception/common/onboard:apollo_perception_common_onboard",
    ],
)

apollo_package()

cpplint()
