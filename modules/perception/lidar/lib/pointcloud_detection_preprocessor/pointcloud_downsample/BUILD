load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

cc_library(
  name = "point_cloud_down_sample",
  srcs = "point_cloud_down_sample.cc",
  hdrs = "point_cloud_down_sample.h",
  deps = [
    "//modules/perception/pipeline:plugin",
  ],
)

cpplint()
