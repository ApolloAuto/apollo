load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "omt_proto",
    deps = [
        ":omt_proto_lib",
    ],
)

proto_library(
    name = "omt_proto_lib",
    srcs = ["omt.proto"],
)

cc_library(
    name = "track_object",
    srcs = ["track_object.cc"],
    hdrs = ["track_object.h"],
    deps = [
        ":frame_list",
        "//modules/perception/base",
        "//modules/perception/inference/utils:inference_cuda_util_lib",
        "//modules/perception/inference/utils:inference_util_lib",
    ],
)

cc_library(
    name = "target",
    srcs = ["target.cc"],
    hdrs = ["target.h"],
    deps = [
        ":frame_list",
        ":omt_proto",
        ":track_object",
        "//modules/perception/base",
        "//modules/perception/camera/common",
        "//modules/perception/camera/lib/obstacle/tracker/common",
    ],
)

cc_library(
    name = "frame_list",
    hdrs = ["frame_list.h"],
    deps = [
        "//cyber",
        "//modules/perception/camera/common",
        "//modules/perception/camera/lib/interface",
        "//modules/perception/inference/utils:inference_cuda_util_lib",
        "//modules/perception/inference/utils:inference_util_lib",
    ],
)

cc_library(
    name = "omt_obstacle_tracker",
    srcs = ["omt_obstacle_tracker.cc"],
    hdrs = ["omt_obstacle_tracker.h"],
    deps = [
        ":frame_list",
        ":obstacle_reference",
        ":omt_proto",
        ":target",
        "//cyber/common:file",
        "//modules/perception/camera/common",
        "//modules/perception/camera/lib/interface",
        "//modules/perception/camera/lib/obstacle/tracker/common",
    ],
)

cc_library(
    name = "obstacle_reference",
    srcs = ["obstacle_reference.cc"],
    hdrs = ["obstacle_reference.h"],
    deps = [
        ":frame_list",
        ":omt_proto",
        ":target",
        "//cyber",
        "//modules/perception/camera/common",
        "//modules/perception/camera/lib/interface",
    ],
)

cpplint()
