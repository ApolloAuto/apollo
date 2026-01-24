load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")
load("@local_config_cuda//cuda:build_defs.bzl", "cuda_library")
load("//tools/platform:build_defs.bzl", "if_profiler")

package(default_visibility = ["//visibility:public"])

PERCEPTION_COPTS = ['-DMODULE_NAME=\\"perception\\"']

filegroup(
    name = "camera_detection_multi_stage_files",
    srcs = glob([
        "conf/**",
        "dag/**",
        "data/**",
    ]),
)

apollo_cc_library(
    name = "apollo_perception_camera_detection_multi_stage",
    srcs = [
        "detector/yolo/postprocess.cc",
        "detector/yolo/yolo_obstacle_detector.cc",
        "detector/yolox3d/postprocess.cc",
        "detector/yolox3d/yolox3d_obstacle_detector.cc",
    ],
    hdrs = [
        "detector/yolo/postprocess.h",
        "detector/yolo/yolo_obstacle_detector.h",
        "detector/yolox3d/postprocess.h",
        "detector/yolox3d/yolox3d_obstacle_detector.h",
    ],
    copts = PERCEPTION_COPTS + if_profiler(),
    deps = [
        "//cyber",
        ":yolo_postprocess_cuda",
        "//modules/common_msgs/sensor_msgs:sensor_image_cc_proto",
        "//modules/perception/camera_detection_multi_stage/detector/yolox3d/proto:yolox3d_model_param_cc_proto",
        "//modules/perception/camera_detection_multi_stage/proto:camera_detection_multi_stage_cc_proto",
        "//modules/perception/common:perception_common_util",
        "//modules/perception/common/algorithm:apollo_perception_common_algorithm",
        "//modules/perception/common/base:apollo_perception_common_base",
        "//modules/perception/common/camera:apollo_perception_common_camera",
        "//modules/perception/common/inference:apollo_perception_common_inference",
        "//modules/perception/common/lib:apollo_perception_common_lib",
        "//modules/perception/common/onboard:apollo_perception_common_onboard",
        "//modules/perception/common/proto:model_info_cc_proto",
        "//modules/perception/common/interface:base_obstacle_detector",
        "@com_google_protobuf//:protobuf",
        "@eigen",
        "@opencv//:imgproc",
        "@local_config_cuda//cuda:cublas",
        "@local_config_cuda//cuda:cuda_headers",
        "@local_config_cuda//cuda:cudart",
        "@local_config_cuda//cuda:cudnn",
        "@local_config_cuda//cuda:cudnn_header",
        "@local_config_tensorrt//:tensorrt",
    ],
)

cuda_library(
    name = "yolo_postprocess_cuda",
    srcs = [
        "detector/yolo/postprocess.cu",
        "detector/yolo/object_maintainer.cc",
    ],
    hdrs = [
        "detector/yolo/postprocess.h",
        "detector/yolo/object_maintainer.h",
    ],
    deps = [
        "//modules/perception/camera_detection_multi_stage/detector/yolo/proto:model_param_cc_proto",
        "//modules/perception/common/base:apollo_perception_common_base",
        "//modules/perception/common/proto:model_info_cc_proto",
        "@local_config_cuda//cuda:cudart",
    ],
)

apollo_component(
    name = "libcamera_detection_multi_stage_component_camera.so",
    srcs = [
        "camera_detection_multi_stage_component.cc",
    ],
    hdrs = [
        "camera_detection_multi_stage_component.h",
    ],
    copts = PERCEPTION_COPTS + if_profiler(),
    deps = [
        "//cyber",
        ":apollo_perception_camera_detection_multi_stage",
        "//modules/common_msgs/sensor_msgs:sensor_image_cc_proto",
        "//modules/perception/camera_detection_multi_stage/proto:camera_detection_multi_stage_cc_proto",
        "//modules/perception/common:perception_common_util",
        "//modules/perception/common/algorithm:apollo_perception_common_algorithm",
        "//modules/perception/common/camera:apollo_perception_common_camera",
        "//modules/perception/common/onboard:apollo_perception_common_onboard",
        "//modules/perception/common/proto:model_info_cc_proto",
        "@local_config_cuda//cuda:cublas",
        "@local_config_cuda//cuda:cuda_headers",
        "@local_config_cuda//cuda:cudart",
        "@local_config_cuda//cuda:cudnn",
        "@local_config_cuda//cuda:cudnn_header",
        "@local_config_tensorrt//:tensorrt",
    ],
)

apollo_package(enable_source=False)

cpplint()
