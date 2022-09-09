load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
  name = "plugin_factory",
  srcs = "plugin_factory.cc",
  hdrs = [
    "plugin.h",
    "plugin_factory.h",
  ],
  deps = [
    "modules/common/util:util_tool",
  ],
)

cc_library(
  name = "stage",
  srcs = "stage.cc",
  hdrs = [
    "plugin.h",
    "data_frame.h",
    "stage.h",
  ],
  deps = [
    ":plugin_factory",
    "//modules/perception/pipeline/proto:pipeline_config_cc_proto",
  ],
)

cc_library(
  name = "pipeline",
  srcs = "pipeline.cc",
  hdrs = "pipeline.h",
  deps = [
    ":stage",
    "//modules/perception/pipeline/proto:pipeline_config_cc_proto",
    "//modules/perception/lidar/lib/detector/point_pillars_detection",
    "//modules/perception/lidar/lib/map_manager",
    "//modules/perception/lidar/lib/object_builder",
    "//modules/perception/lidar/lib/object_filter_bank",
    "//modules/perception/lidar/lib/pointcloud_preprocessor",
  ],
)

cpplint()
