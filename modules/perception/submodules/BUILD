load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

PERCEPTION_COPTS = ['-DMODULE_NAME=\\"perception\\"']

cc_binary(
    name = "libperception_submodule_lidar.so",
    linkshared = True,
    linkstatic = False,
    deps = [":perception_submodule_inner_lidar"],
)

cc_library(
    name = "perception_submodule_inner_lidar",
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        ":lidar_detection_submodule",
    ],
    alwayslink = True,
)

cc_library(
    name = "lidar_detection_submodule",
    srcs = ["lidar_detection_submodule.cc"],
    hdrs = ["lidar_detection_submodule.h"],
    deps = [
        ":common_flags",
        ":lidar_inner_submodule_messages",
        ":transform_wrapper",
        "//cyber/time:clock",
        "//modules/common/util:string_util",
        "//modules/perception/common/sensor_manager",
        "//modules/perception/lib/registerer",
        "//modules/perception/lidar/app:lidar_obstacle_detection",
        "//modules/perception/lidar/common",
        "//modules/perception/proto:lidar_submodule_config_cc_proto",
        "@eigen",
    ],
)

cc_library(
    name = "common_flags",
    srcs = ["common_flags.cc"],
    hdrs = ["common_flags.h"],
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "inner_submodule_messages",
    hdrs = ["inner_submodule_messages.h"],
)

cc_library(
    name = "lidar_inner_submodule_messages",
    hdrs = [
        "lidar_inner_submodule_messages.h",
    ],
    deps = [
        ":inner_submodule_messages",
        "//cyber",
        "//modules/perception/lidar/common",
    ],
)

cc_library(
    name = "transform_wrapper",
    srcs = ["transform_wrapper.cc"],
    hdrs = ["transform_wrapper.h"],
    deps = [
        "//modules/common/util:string_util",
        "//modules/perception/common/sensor_manager",
        "//modules/transform:buffer",
        "@com_github_gflags_gflags//:gflags",
        "@eigen",
    ],
)

cpplint()
