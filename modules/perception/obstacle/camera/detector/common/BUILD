load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "feature_extractor",
    srcs = ["feature_extractor.cc"],
    hdrs = ["feature_extractor.h"],
    deps = [
        "//modules/common:log",
        "//modules/perception/common:pcl_util",
        "//modules/perception/cuda_util",
        "//modules/perception/lib/base",
        "//modules/perception/obstacle/base",
        "//modules/perception/obstacle/camera/common:util",
        "//modules/perception/obstacle/camera/detector/common/proto:tracking_feature_proto",
        "@caffe//:lib",
        "@cuda",
        "@eigen",
        "@opencv2//:core",
        "@opencv2//:imgproc",
    ],
)

cpplint()
