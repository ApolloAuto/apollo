load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "radar_lib_interface",
    srcs = [
    ],
    hdrs = [
        "base_detector.h",
        "base_filter.h",
        "base_matcher.h",
        "base_preprocessor.h",
        "base_radar_obstacle_perception.h",
        "base_roi_filter.h",
        "base_tracker.h",
    ],
    deps = [
        "//modules/drivers/proto:sensor_proto",
    ],
)

cpplint()
