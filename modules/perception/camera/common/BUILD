load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "camera_frame",
    hdrs = ["camera_frame.h"],
    deps = [
        ":data_provider",
        "//modules/perception/base",
    ],
)

cc_library(
    name = "camera_ground_plane",
    srcs = ["camera_ground_plane.cc"],
    hdrs = ["camera_ground_plane.h"],
    deps = [
        "//modules/perception/common/i_lib/algorithm:i_sort",
        "//modules/perception/common/i_lib/core",
        "//modules/perception/common/i_lib/da:i_ransac",
        "//modules/perception/common/i_lib/geometry:i_plane",
    ],
)

cc_library(
    name = "data_provider",
    srcs = ["data_provider.cc"],
    hdrs = ["data_provider.h"],
    deps = [
        ":undistortion_handler",
        "//modules/perception/base",
    ],
)

cc_library(
    name = "global_config",
    hdrs = ["global_config.h"],
)

cc_library(
    name = "math_functions",
    hdrs = [
        "math_functions.h",
        "//modules/perception/base:base",
    ],
)

cc_library(
    name = "object_template_manager",
    srcs = ["object_template_manager.cc"],
    hdrs = ["object_template_manager.h"],
    deps = [
        "//cyber",
        "//modules/perception/base",
        "//modules/perception/camera/common/proto:object_template_meta_schema_proto",
        "//modules/perception/common/io:io_util",
        "//modules/perception/lib/config_manager",
        "//modules/perception/lib/thread",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "timer",
    hdrs = ["timer.h"],
)

cc_library(
    name = "twod_threed_util",
    hdrs = ["twod_threed_util.h"],
    deps = [
        "//modules/perception/common/i_lib/core",
        "//modules/perception/common/i_lib/geometry:i_plane",
        "@eigen",
    ],
)

cc_library(
    name = "undistortion_handler",
    srcs = ["undistortion_handler.cc"],
    hdrs = ["undistortion_handler.h"],
    deps = [
        "//modules/perception/base",
        "//modules/perception/base:blob",
        "//modules/perception/common/sensor_manager",
    ],
)

cc_library(
    name = "util",
    srcs = ["util.cc"],
    hdrs = ["util.h"],
    deps = [
        "//modules/perception/base",
    ],
)

cc_library(
    name = "lane_object",
    hdrs = ["lane_object.h"],
)

cc_library(
    name = "common",
    deps = [
        ":camera_frame",
        ":camera_ground_plane",
        ":data_provider",
        ":global_config",
        #        ":lane_object",
        ":math_functions",
        ":object_template_manager",
        ":timer",
        ":twod_threed_util",
        ":undistortion_handler",
        ":util",
    ],
)

cpplint()
