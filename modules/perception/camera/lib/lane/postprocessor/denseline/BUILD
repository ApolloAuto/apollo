load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])


cc_proto_library(
    name = "denseline_postprocessor_proto",
    deps = [
        ":denseline_postprocessor_proto_lib",
    ],
)

proto_library(
    name = "denseline_postprocessor_proto_lib",
    srcs = [
        "denseline_postprocessor.proto",
    ],
)


cc_library(
    name = "denseline_lane_postprocessor",
    srcs = [
        "denseline_lane_postprocessor.cc",
    ],
    hdrs = [ 
        "denseline_lane_postprocessor.h",
    ],

    deps = [
        "//cybertron",
        ":denseline_postprocessor_proto",
        "//modules/common/util:file_util",
        "//modules/perception/base:base",
        "//modules/perception/camera/common:common",
        "//modules/perception/camera/lib/interface:interface",
        "//modules/perception/camera/lib/lane/common:common_functions",
        "//modules/perception/camera/lib/lane/common:denseline_proto",
        "//modules/perception/lib/io:file_util",
        "//modules/perception/lib/registerer:registerer",
        "//modules/perception/lib/singleton:singleton",
        "//modules/perception/lib/utils:utils",
    ],
)

cpplint()
