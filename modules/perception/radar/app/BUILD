load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "radar_obstacle_perception",
    srcs = [
        "radar_obstacle_perception.cc",
    ],
    hdrs = [
        "radar_obstacle_perception.h",
    ],
    deps = [
        "//modules/perception/base:base",
        "//modules/perception/lib/singleton:singleton",
        "//modules/perception/lib/config_manager:config_manager",
        "//modules/perception/lib/registerer:registerer",
        "//modules/perception/lib/utils:utils",
        "//modules/perception/common/geometry:roi_filter",
        "//modules/perception/radar/common:types",
        "//modules/perception/radar/lib/interface:base_radar_obstacle_perception",
        "//modules/perception/radar/lib/interface:base_detector",
        "//modules/perception/radar/lib/interface:base_preprocessor",
        "//modules/perception/radar/lib/interface:base_roi_filter",
        "//modules/perception/radar/lib/interface:base_tracker",
        "//modules/perception/proto:perception_config_schema_proto",
        "//modules/drivers/proto:sensor_proto",
        "@eigen",
    ],
)

cpplint()
