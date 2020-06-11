load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "lane_based_calibrator",
    srcs = ["lane_based_calibrator.cc"],
    hdrs = ["lane_based_calibrator.h"],
    deps = [
        ":lane_struct_for_calib",
        "//modules/perception/camera/lib/calibrator/common:histogram_estimator",
        "//modules/perception/common/i_lib/core",
    ],
)

cc_library(
    name = "lane_struct_for_calib",
    hdrs = ["lane_struct_for_calib.h"],
    deps = [
        "@eigen",
    ],
)

cc_library(
    name = "laneline_calibrator",
    srcs = ["laneline_calibrator.cc"],
    hdrs = ["laneline_calibrator.h"],
    deps = [
        ":lane_based_calibrator",
        "//modules/perception/camera/lib/interface",
        "//modules/perception/common/i_lib/core",
    ],
)

cpplint()
