load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

cc_library(
  name = "point_cloud_detection_postprocessor",
  srcs = [
    "point_cloud_detection_postprocessor.cc",
  ],
  hdrs = [
    "point_cloud_detection_postprocessor.h",
  ],
  deps = [
    "//modules/perception/pipeline:stage",
  ],
)

cpplint()
