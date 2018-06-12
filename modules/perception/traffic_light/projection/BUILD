load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "perception_traffic_light_projection",
    srcs = [
        "multi_camera_projection.cc",
        "projection.cc",
    ],
    hdrs = [
        "multi_camera_projection.h",
        "projection.h",
    ],
    deps = [
        "//modules/common:log",
        "//modules/common/proto:error_code_proto",
        "//modules/common/proto:header_proto",
        "//modules/map/proto:map_proto",
        "//modules/perception/lib/config_manager",
        "//modules/perception/proto:perception_proto",
        "//modules/perception/proto/traffic_light:multi_camera_projection_config_lib_proto",
        "//modules/perception/traffic_light/base",
        "//modules/perception/traffic_light/interface",
    ],
)

cc_test(
    name = "multi_camera_projection_test",
    size = "small",
    srcs = [
        "multi_camera_projection_test.cc",
    ],
    data = [
        "//modules/perception:perception_data",
        "//modules/perception:perception_model",
        "//modules/perception/conf:perception_config",
    ],
    deps = [
        "perception_traffic_light_projection",
        "//modules/perception/lib/base",
        "@gtest//:main",
    ],
)

cpplint()
