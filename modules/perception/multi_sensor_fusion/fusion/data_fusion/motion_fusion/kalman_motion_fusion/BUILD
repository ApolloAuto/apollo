load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "kalman_motion_fusion",
    srcs = ["kalman_motion_fusion.cc"],
    hdrs = ["kalman_motion_fusion.h"],
    deps = [
        "//modules/common/util:util_tool",
        "//modules/perception/common/base",
        "//modules/perception/common:perception_gflags",
        "//modules/perception/common/algorithm/geometry:basic",
        "//modules/perception/common/algorithm/sensor_manager",
        "//modules/perception/multi_sensor_fusion/base",
        "//modules/perception/multi_sensor_fusion/common:base_filter",
        "//modules/perception/multi_sensor_fusion/common:kalman_filter",
        "//modules/perception/multi_sensor_fusion/interface",
        "//modules/perception/common/lib/registerer",
    ],
)

# todo(zero): need fix
# cc_test(
#     name = "kalman_motion_fusion_test",
#     size = "small",
#     srcs = ["kalman_motion_fusion_test.cc"],
#     deps = [
#         ":kalman_motion_fusion",
#         "@com_google_googletest//:gtest_main",
#     ],
# )

apollo_package()
cpplint()
