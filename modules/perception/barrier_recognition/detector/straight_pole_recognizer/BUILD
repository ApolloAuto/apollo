load("//tools:apollo_package.bzl", "apollo_package", "apollo_cc_library", "apollo_plugin")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/platform:build_defs.bzl", "if_profiler")

package(default_visibility = ["//visibility:public"])

PERCEPTION_COPTS = ['-DMODULE_NAME=\\"perception\\"']

apollo_plugin(
    name = "libstraight_pole_recognizer.so",
    description = ":plugins.xml",
    deps = [":lib_spr"],
)

apollo_cc_library(
    name = "lib_spr",
    srcs = ["straight_pole_recognizer.cc"],
    hdrs = ["straight_pole_recognizer.h"],
    copts = PERCEPTION_COPTS + if_profiler() + ["-DENABLE_PROFILER=1"],
    deps = [
        "//cyber",
        "//modules/perception/common:perception_common_util",
        "//modules/perception/common/algorithm:apollo_perception_common_algorithm",
        "//modules/perception/common/lidar:apollo_perception_common_lidar",
        "//modules/perception/common/lib:apollo_perception_common_lib",
        "//modules/perception/barrier_recognition:apollo_perception_barrier_recognition",
        "//modules/perception/barrier_recognition/detector/proto:straight_pole_barrier_config_cc_proto",
    ],
)

apollo_package()
cpplint()
