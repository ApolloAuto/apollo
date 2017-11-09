load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "perception_traffic_light_recognizer",
    srcs = [
        "classify.cc",
        "unity_recognize.cc",
    ],
    hdrs = [
        "classify.h",
        "unity_recognize.h",
    ],
    deps = [
        "//modules/common:log",
        "//modules/perception/lib/config_manager",
    ],
)

cpplint()
