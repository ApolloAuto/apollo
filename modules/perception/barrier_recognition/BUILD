load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/platform:build_defs.bzl", "if_profiler")
load("//third_party/gpus:common.bzl", "gpu_library", "if_cuda", "if_rocm")

package(default_visibility = ["//visibility:public"])

PERCEPTION_COPTS = ['-DMODULE_NAME=\\"perception\\"']

filegroup(
    name = "barrier_recognition_files",
    srcs = glob([
        "conf/**",
        "dag/**",
        "data/**",
        "launch/**",
    ])
)

apollo_cc_library(
    name = "apollo_perception_barrier_recognition",
    srcs = [
        "detector/barrier_recognition_bank.cc",
        "tracker/barrier_status_tracker.cc",
    ],
    hdrs = [
        "interface/base_barrier_recognizer.h",
        "detector/barrier_recognition_bank.h",
        "tracker/barrier_status_tracker.h",
    ],
    copts = PERCEPTION_COPTS + if_profiler(),
    deps = [
        "//cyber",
        "//modules/common_msgs/sensor_msgs:pointcloud_cc_proto",
        "//modules/common_msgs/perception_msgs:perception_obstacle_cc_proto",
        "//modules/common_msgs/perception_msgs:perception_barrier_gate_cc_proto",
        "//modules/perception/common/base:apollo_perception_common_base",
        "//modules/perception/common/lib:apollo_perception_common_lib",
        "//modules/perception/common/lidar:apollo_perception_common_lidar",
        "//modules/perception/common/onboard:apollo_perception_common_onboard",
        
        "//modules/perception/common/hdmap:apollo_perception_common_hdmap",

        "//modules/perception/barrier_recognition/detector/proto:barrier_bank_config_cc_proto",
        "//modules/perception/barrier_recognition/detector/proto:straight_pole_barrier_config_cc_proto",
        "//modules/perception/barrier_recognition/tracker/proto:barrier_tracker_config_cc_proto",
        "//modules/perception/barrier_recognition/proto:barrier_recognition_component_config_cc_proto",
    ]
)

apollo_component(
    name = "libbarrier_recognition_component.so",
    srcs = [
        "barrier_recognition_component.cc",
    ],
    hdrs = [
        "barrier_recognition_component.h",
    ],
    copts = PERCEPTION_COPTS + if_profiler(),
    linkopts = ["-lleveldb"],
    deps = [
        ":apollo_perception_barrier_recognition",
        "//cyber",
        "//modules/common_msgs/perception_msgs:perception_barrier_gate_cc_proto",
        "//modules/perception/common/onboard:apollo_perception_common_onboard",
        "//modules/perception/barrier_recognition/proto:barrier_recognition_component_config_proto",
    ],
)

apollo_package(enable_source=False)
cpplint()
