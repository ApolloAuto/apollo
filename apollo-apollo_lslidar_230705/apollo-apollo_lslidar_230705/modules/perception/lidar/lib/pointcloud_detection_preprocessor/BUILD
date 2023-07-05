load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
  name = "pointcloud_detection_preprocessor",
  srcs = [
    "pointcloud_detection_preprocessor.cc",
  ],
  hdrs = [
    "pointcloud_detection_preprocessor.h",
  ],
  deps = [
    "//modules/perception/lidar/lib/pointcloud_detection_preprocessor/pointcloud_down_sample",
    "//modules/perception/pipeline:plugin_factory",
    "//modules/perception/pipeline:stage",
    "//modules/perception/pipeline/proto/stage:pointcloud_detection_preprocessor_config_cc_proto",
  ],
)

cpplint()
