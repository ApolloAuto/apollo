load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "config_manager",
    srcs = [
            "config_manager.cc",
            "calibration_config_manager.cc",
            ],
    hdrs = [
            "config_manager.h",
            "calibration_config_manager.h",
            ],
    deps = [
        "//modules/common:log",
        "//modules/common/util",
        "//modules/perception/common:perception_common",
        "//modules/perception/lib/base",
        "//modules/perception/lib/config_manager/proto:config_proto",
        "//modules/perception/obstacle/camera/common:common",
        "@undistort//:undistort",
        "@yaml_cpp//:yaml",
        "@opencv2//:imgproc",
        "@cuda",
    ],
)

cc_test(
    name = "config_manager_test",
    size = "small",
    srcs = ["config_manager_test.cc"],
    data = ["//modules/perception:perception_data"],
    deps = [
        ":config_manager",
        "//modules/perception/common:perception_common",
        "@gtest//:main",
    ],
)

cpplint()
