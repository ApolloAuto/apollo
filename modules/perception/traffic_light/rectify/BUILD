load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "perception_traffic_light_rectify",
    srcs = [
        "cropbox.cc",
        "detection.cc",
        "munkres.cc",
        "select.cc",
        "unity_rectify.cc",
    ],
    hdrs = [
        "cropbox.h",
        "detection.h",
        "munkres.h",
        "select.h",
        "unity_rectify.h",
    ],
    deps = [
        "//modules/common:log",
        "//modules/perception/lib/config_manager",
    ],
)

cpplint()
