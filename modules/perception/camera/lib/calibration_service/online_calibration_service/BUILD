load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "online_calibration_service",
    srcs = ["online_calibration_service.cc"],
    hdrs = ["online_calibration_service.h"],
    deps = [
        "//modules/perception/camera/lib/interface",
        "//modules/perception/common/i_lib/core",
        "//modules/perception/common/i_lib/geometry:i_plane",
    ],
)

cpplint()
