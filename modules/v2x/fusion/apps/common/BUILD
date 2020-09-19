load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "ft_definitions",
    hdrs = ["ft_definitions.h"],
    deps = [
        "//modules/localization/proto:localization_cc_proto",
        "//modules/perception/proto:perception_camera_cc_proto",
        "//modules/v2x/fusion/libs/common:v2x_object",
        "//modules/v2x/proto:v2x_obstacles_cc_proto",
    ],
)

cc_library(
    name = "trans_tools",
    srcs = ["trans_tools.cc"],
    hdrs = ["trans_tools.h"],
    copts = ['-DMODULE_NAME=\\"v2x_fusion\\"'],
    deps = [
        ":ft_definitions",
    ],
)

cpplint()
