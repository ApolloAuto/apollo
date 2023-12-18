load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

apollo_cc_library(
    name = "apollo_perception_tools_common",
    hdrs = [
        "ground_truth.h",
        "util.h",
        "visualizer.h",
    ],
    srcs = [
        "ground_truth.cc",
        "util.cc",
        "visualizer.cc",
    ],
    deps = [
        "@opencv//:imgproc",
        "//cyber",
        "//modules/perception/common/algorithm:apollo_perception_common_algorithm",
        "//modules/perception/common/base:apollo_perception_common_base",
        "//modules/perception/common/onboard:apollo_perception_common_onboard",
    ],
)

apollo_package()
cpplint()
