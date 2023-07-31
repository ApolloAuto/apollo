load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "lane_based_calibrator",
    srcs = ["lane_based_calibrator.cc"],
    alwayslink = True,
    hdrs = ["lane_based_calibrator.h"],
    deps = [
        ":lane_struct_for_calib",
        "//modules/perception/common/algorithm/i_lib/core",
        "//modules/perception/common/lib/calibrator/common:histogram_estimator",
    ],
)

cc_library(
    name = "lane_struct_for_calib",
    hdrs = ["lane_struct_for_calib.h"],
    alwayslink = True,
    deps = [
        "//modules/common/util:util_tool",
        "@eigen",
    ],
)

cc_library(
    name = "laneline_calibrator",
    srcs = ["laneline_calibrator.cc"],
    hdrs = ["laneline_calibrator.h"],
    deps = [
        ":lane_based_calibrator",
        "//modules/perception/common/algorithm/i_lib/core",
        "//modules/perception/common/lib/interface:base_calibrator",
    ],
    alwayslink = True,
)

apollo_package()
cpplint()
