load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "util",
    srcs = ["util.cc"],
    hdrs = ["util.h"],
    deps = [
        "@caffe//:lib",
        "@eigen//:eigen",
        "@opencv2//:core",
        "@undistort//:undistort",
        "//modules/common:log",
        "@opencv2//:imgproc",
        "@cuda",
    ]
)

cc_library(
    name = "feature_extractor",
    srcs = ["feature_extractor.cc"],
    hdrs = ["feature_extractor.h"],
    deps = [
        "@caffe//:lib",
        "@eigen//:eigen",
        "@opencv2//:core",
        "@undistort//:undistort",
        "//modules/common:log",
        "//modules/perception/lib/base:base",
        "@opencv2//:imgproc",
        "@cuda",
        "//modules/perception/lib/pcl_util:pcl_util",
        "//modules/perception/obstacle/camera/common:common",
        "//modules/perception/obstacle/base:perception_obstacle_base",
        "//modules/perception/obstacle/camera/detector/common/proto:tracking_feature_proto",
    ]
)


cpplint()
