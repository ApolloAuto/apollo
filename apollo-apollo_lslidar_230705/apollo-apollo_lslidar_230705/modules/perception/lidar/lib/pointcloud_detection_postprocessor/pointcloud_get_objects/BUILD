load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
  name = "pointcloud_get_objects",
  srcs = [
    "pointcloud_get_objects.cc",
  ],
  hdrs = [
    "pointcloud_get_objects.h",
  ],
  deps = [
    "//modules/perception/base:object",
    "//modules/perception/base:point",
    "//modules/perception/common:perception_gflags",
    "//modules/perception/pipeline:data_frame",
    "//modules/perception/pipeline:plugin",
  ],
)

cpplint()
