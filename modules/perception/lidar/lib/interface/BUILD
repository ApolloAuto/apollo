load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "base_segmentation",
    hdrs = [
        "base_segmentation.h",
    ],
    deps = [
        "//modules/perception/lib/registerer",
        "//modules/perception/lidar/common:lidar_frame",
    ],
)

cc_library(
    name = "base_classifier",
    hdrs = [
        "base_classifier.h",
    ],
    deps = [
        "//modules/perception/lib/registerer",
        "//modules/perception/lidar/common:lidar_frame",
    ],
)

cpplint()
