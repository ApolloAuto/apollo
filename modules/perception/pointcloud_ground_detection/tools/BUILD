load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_package", "apollo_cc_binary")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

apollo_cc_binary(
    name = "offline_pointcloud_ground_detection",
    srcs = ["offline_pointcloud_ground_detection.cc"],
    deps = [
        "//cyber",
        "//modules/perception/common/algorithm:apollo_perception_common_algorithm",
        "//modules/perception/common/lidar:apollo_perception_common_lidar",
        "//modules/perception/pointcloud_ground_detection:apollo_perception_pointcloud_ground_detection",
        "@eigen",
        "@local_config_pcl//:pcl",
    ],
)

apollo_package()
cpplint()
