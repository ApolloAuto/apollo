load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "camera_cipv",
    srcs = ["cipv.cc"],
    hdrs = ["cipv.h"],
    deps = [
        "//modules/common:log",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/perception/lib/base",
        "//modules/perception/obstacle/base",
        "//modules/perception/obstacle/camera/common",
        "//modules/perception/obstacle/camera/lane_post_process/common:type",
        "//modules/perception/obstacle/common",
        "@eigen",
    ],
)

cpplint()
