load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_package", "apollo_cc_binary")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

apollo_cc_binary(
    name = "offline_lidar_cpdet_detection",
    srcs = ["offline_lidar_cpdet_detection.cc"],
    deps = [
        "//cyber",
        "//modules/perception/common/algorithm:apollo_perception_common_algorithm",
        "//modules/perception/common/lidar:apollo_perception_common_lidar",
        "//modules/perception/lidar_cpdet_detection:apollo_perception_lidar_cpdetection",
        "@eigen",
    ],
)

apollo_package()
cpplint()
