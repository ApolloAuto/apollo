load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "base",
    srcs = [
        "image.cc",
        "light.cc",
        "pose.cc",
        "tl_shared_data.cc",
        "utils.cc",
    ],
    hdrs = [
        "image.h",
        "image_lights.h",
        "light.h",
        "pose.h",
        "tl_shared_data.h",
        "utils.h",
    ],
    deps = [
        "//modules/common:log",
        "//modules/map/proto:map_proto",
        "//modules/perception/lib/config_manager",
        "//modules/perception/onboard",
        "//modules/perception/proto:perception_proto",
        "//modules/perception/traffic_light/util",
    ],
)

cc_test(
    name = "base_test",
    size = "small",
    srcs = [
        "image_test.cc",
        "light_test.cc",
        "utils_structs_test.cc",
    ],
    deps = [
        "//modules/common:log",
        "//modules/map/proto:map_proto",
        "//modules/perception/lib/config_manager",
        "//modules/perception/proto:perception_proto",
        "//modules/perception/traffic_light/base",
        "//modules/perception/traffic_light/util",
        "@gtest//:main",
        "@opencv2//:core",
    ],
)

cpplint()
