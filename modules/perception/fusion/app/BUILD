load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "obstacle_multi_sensor_fusion",
    srcs = ["obstacle_multi_sensor_fusion.cc"],
    hdrs = ["obstacle_multi_sensor_fusion.h"],
    deps = [
        "//cyber",
        "//modules/perception/base",
        "//modules/perception/fusion/base",
        "//modules/perception/fusion/lib/interface:base_multisensor_fusion",
        "//modules/perception/fusion/lib/interface:base_fusion_system",
        "//modules/perception/fusion/lib/fusion_system/probabilistic_fusion:probabilistic_fusion",
        "//modules/perception/lib/registerer",
        "@eigen",
    ],
    alwayslink = True,
)

cpplint()
