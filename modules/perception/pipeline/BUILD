load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
  name = "data_frame",
  hdrs = [
    "data_frame.h",
  ],
  deps = [
    "//modules/perception/camera/common:camera_frame",
    "//modules/perception/fusion/base:fusion_frame",
    "//modules/perception/lidar/common:lidar_frame",
  ],
)

cc_library(
  name = "plugin_factory",
  srcs = [
    "plugin_factory.cc",
  ],
  hdrs = [
    "plugin_factory.h",
  ],
  deps = [
    ":plugin",
    "//modules/common/util:util_tool",
    "//modules/perception/pipeline/proto:pipeline_config_cc_proto",
    "//modules/perception/lidar/lib/object_filter_bank/roi_boundary_filter",
  ],
)

cc_library(
  name = "plugin",
  hdrs = [
    "plugin.h",
  ],
  deps = [
    "//modules/perception/pipeline/proto:pipeline_config_cc_proto",
  ],
)

cc_library(
  name = "stage",
  srcs = [
    "stage.cc",
  ],
  hdrs = [
    "stage.h",
  ],
  deps = [
    ":data_frame",
    ":plugin_factory",
    ":plugin",
    "//modules/perception/pipeline/proto:pipeline_config_cc_proto",
  ],
)

cc_library(
  name = "pipeline",
  srcs = [
    "pipeline.cc",
  ],
  hdrs = [
    "pipeline.h",
  ],
  deps = [
    ":stage",
    "//modules/perception/pipeline/proto:pipeline_config_cc_proto",
    "//modules/perception/lidar/lib/detector/point_pillars_detection",
    "//modules/perception/lidar/lib/map_manager",
    "//modules/perception/lidar/lib/object_builder",
    "//modules/perception/lidar/lib/object_filter_bank",
    "//modules/perception/lidar/lib/pointcloud_preprocessor",
    "//modules/perception/camera/lib/traffic_light/detector/detection",
    "//modules/perception/camera/lib/traffic_light/detector/recognition",
    "//modules/perception/camera/lib/traffic_light/tracker:semantic_decision",
  ],
)

cpplint()
