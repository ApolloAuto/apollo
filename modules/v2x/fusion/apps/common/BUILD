load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "apps_common",
    srcs = [
        "trans_tools.cc",
    ],
    hdrs = [
        "ft_definitions.h",
        "trans_tools.h",
    ],
    copts = ['-DMODULE_NAME=\\"v2x_fusion\\"'],
    deps = [
        "//cyber",
        "//modules/common/math",
        "//modules/localization/proto:localization_cc_proto",
        "//modules/perception/proto:perception_camera_cc_proto",
        "//modules/v2x/fusion/libs/common/base",
        "//modules/v2x/proto:v2x_obstacles_cc_proto",
    ],
)

cpplint()
