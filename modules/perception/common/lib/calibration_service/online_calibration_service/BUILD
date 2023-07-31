load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "online_calibration_service",
    srcs = ["online_calibration_service.cc"],
    hdrs = ["online_calibration_service.h"],
    deps = [
        "//modules/perception/common/algorithm/i_lib/core",
        "//modules/perception/common/algorithm/i_lib/geometry:i_plane",
        "//modules/perception/common/lib/calibrator/laneline:laneline_calibrator",
        "//modules/perception/common/lib/interface:base_calibration_service",
        "//modules/perception/common/lib/interface:base_calibrator",
    ],
    alwayslink = True,
)

apollo_package()
cpplint()
