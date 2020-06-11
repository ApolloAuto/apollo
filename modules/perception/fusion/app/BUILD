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
        "//modules/perception/fusion/lib/interface",
        "//modules/perception/lib/registerer",
        "@eigen",
    ],
)

cpplint()
