load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
  name = "pointcloud_detection_postprocessor",
  srcs = [
    "pointcloud_detection_postprocessor.cc",
  ],
  hdrs = [
    "pointcloud_detection_postprocessor.h",
  ],
  deps = [
    "//modules/perception/pipeline:stage",
    "//modules/perception/pipeline:plugin_factory",
    "//modules/perception/lidar/lib/pointcloud_detection_postprocessor/pointcloud_get_objects",
  ],
)

cpplint()
