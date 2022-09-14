load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

cc_library(
  name = "pointcloud_detection_preprocessor",
  srcs = "pointcloud_detection_preprocessor.cc",
  hdrs = "pointcloud_detection_preprocessor.h",
  deps = [
    "//modules/perception/pipeline:stage",
  ],
)

cpplint()
