load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
  name = "pointcloud_down_sample",
  srcs = [
    "pointcloud_down_sample.cc",
  ],
  hdrs = [
    "pointcloud_down_sample.h",
  ],
  deps = [
    "//modules/perception/base",
    "//modules/perception/common:perception_gflags",
    "//modules/perception/lidar/common",
    "//modules/perception/pipeline:data_frame",
    "//modules/perception/pipeline:plugin",
    "//modules/perception/pipeline/proto/plugin:pointcloud_downsample_config_cc_proto",
  ],
)

cpplint()
