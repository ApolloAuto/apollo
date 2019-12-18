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
    srcs = ["denseline_postprocessor.proto"],
)

cc_library(
    name = "denseline_lane_postprocessor",
    srcs = ["denseline_lane_postprocessor.cc"],
    hdrs = ["denseline_lane_postprocessor.h"],
    deps = [
        ":denseline_postprocessor_proto",
        "//cyber",
        "//modules/perception/base",
        "//modules/perception/camera/common",
        "//modules/perception/camera/lib/interface",
        "//modules/perception/camera/lib/lane/common:common_functions",
        "//modules/perception/camera/lib/lane/common:denseline_proto",
        "//modules/perception/lib/registerer",
        "//modules/perception/lib/utils",
    ],
)

cpplint()
