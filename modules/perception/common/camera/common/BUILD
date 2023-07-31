load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "camera_frame",
    hdrs = [
        "camera_frame.h",
    ],
    alwayslink = True,
    deps = [
        ":data_provider",
        "//modules/perception/common/base",
    ],
)

cc_library(
    name = "camera_ground_plane",
    srcs = ["camera_ground_plane.cc"],
    alwayslink = True,
    hdrs = ["camera_ground_plane.h"],
    deps = [
        "//cyber",
        "//modules/perception/common/algorithm/i_lib/algorithm:i_sort",
        "//modules/perception/common/algorithm/i_lib/core",
        "//modules/perception/common/algorithm/i_lib/da:i_ransac",
        "//modules/perception/common/algorithm/i_lib/geometry:i_plane",
    ],
)

cc_library(
    name = "data_provider",
    srcs = ["data_provider.cc"],
    alwayslink = True,
    hdrs = ["data_provider.h"],
    deps = [
        ":undistortion_handler",
        "//cyber",
        "//modules/perception/common/base",
        "@npp",
    ],
)

cc_library(
    name = "global_config",
    alwayslink = True,
    hdrs = ["global_config.h"],
)

cc_library(
    name = "math_functions",
    alwayslink = True,
    hdrs = [
        "math_functions.h",
        "//modules/perception/common/base:base",
    ],
)

cc_library(
    name = "object_template_manager",
    srcs = ["object_template_manager.cc"],
    hdrs = ["object_template_manager.h"],
    alwayslink = True,
    deps = [
        "//cyber",
        "//modules/perception/common/base",
        "//modules/perception/common:perception_gflags",
        "//modules/perception/common:util",
        "//modules/perception/common/camera/common/proto:object_template_meta_schema_cc_proto",
        "//modules/perception/common/algorithm/io:io_util",
        "//modules/perception/common/lib/thread",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "timer",
    hdrs = ["timer.h"],
    alwayslink = True,
)

cc_library(
    name = "twod_threed_util",
    hdrs = ["twod_threed_util.h"],
    alwayslink = True,
    deps = [
        "//cyber",
        "//modules/perception/common/algorithm/i_lib/core",
        "//modules/perception/common/algorithm/i_lib/geometry:i_plane",
    ],
)

cc_library(
    name = "undistortion_handler",
    srcs = ["undistortion_handler.cc"],
    hdrs = ["undistortion_handler.h"],
    alwayslink = True,
    deps = [
        "//cyber",
        "//modules/perception/common/base",
        "//modules/perception/common/algorithm/sensor_manager",
        "@npp",
    ],
)

cc_library(
    name = "util",
    srcs = ["util.cc"],
    hdrs = ["util.h"],
    alwayslink = True,
    deps = [
        "//cyber",
        "//modules/perception/common/base",
    ],
)

cc_library(
    name = "lane_object",
    hdrs = ["lane_object.h"],
    alwayslink = True,
    deps = [
        "//modules/common/util:util_tool",
    ],
)

cc_library(
    name = "common",
    deps = [
        ":camera_frame",
        ":camera_ground_plane",
        ":data_provider",
        ":global_config",
        ":math_functions",
        ":object_template_manager",
        ":timer",
        ":twod_threed_util",
        ":undistortion_handler",
        ":util",
    ],
    alwayslink = True,
)

cc_library(
    name = "trafficlight_frame",
    hdrs = [
        "trafficlight_frame.h",
    ],
    alwayslink = True,
    deps = [
        "//modules/perception/common/camera/common:data_provider",
        "//modules/perception/common/base",
    ],
)

cc_library(
    name = "pose",
    srcs = ["pose.cc"],
    alwayslink = True,
    hdrs = ["pose.h"],
    deps = [
        "//cyber",
        "//modules/common/util:util_tool",
        "//modules/perception/common/base",
    ],
)

cc_library(
    name = "camera_preprocess_general_util",
    srcs = ["camera_preprocess_general_util.cc"],
    hdrs = ["camera_preprocess_general_util.h"],
    alwayslink = True,
    deps = [
        "@opencv//:core",
        "//cyber",
    ],
)

cc_library(
    name = "cipv_camera",
    srcs = ["cipv_camera.cc"],
    hdrs = ["cipv_camera.h"],
    deps = [
        "//cyber",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/math",
        "//modules/perception/common/base",
        "//modules/perception/common/camera/common:lane_object",
        "//modules/perception/common/lib/interface:base_cipv",
    ],
    alwayslink = True,
)

apollo_package()
cpplint()
