load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "base_forward_declaration",
    hdrs = [
        "base_forward_declaration.h",
    ],
)

cc_library(
    name = "fusion_log",
    hdrs = [
        "fusion_log.h",
    ],
)

cc_library(
    name = "base_init_options",
    srcs = [
        "base_init_options.cc",
    ],
    hdrs = [
        "base_init_options.h",
    ],
    deps = [
        "//modules/perception/lib/config_manager",
    ],
)

cc_library(
    name = "scene",
    srcs = [
        "scene.cc",
    ],
    hdrs = [
        "scene.h",
    ],
    deps = [
        ":sensor",
        ":track",
    ],
)

cc_library(
    name = "sensor",
    srcs = [
        "sensor.cc",
        "sensor_frame.cc",
        "sensor_object.cc",
    ],
    hdrs = [
        "sensor.h",
        "sensor_frame.h",
        "sensor_object.h",
    ],
    deps = [
        ":base_forward_declaration",
        "//framework:cybertron",
        "//modules/perception/base",
        "//modules/perception/base:base_type",
        "//modules/perception/base:frame",
        "//modules/perception/base:object",
        "//modules/perception/base:object_pool_types",
        "//modules/perception/common/sensor_manager",
        "//modules/perception/lib/singleton",
        "@eigen",
    ],
)

cc_library(
    name = "sensor_data_manager",
    srcs = [
        "sensor_data_manager.cc",
    ],
    hdrs = [
        "sensor_data_manager.h",
    ],
    deps = [
        ":sensor",
        "//modules/perception/base:frame",
        "//modules/perception/common/sensor_manager",
        "//modules/perception/lib/singleton",
        "//modules/perception/lib/thread",
    ],
)

cc_library(
    name = "track",
    srcs = [
        "track.cc",
    ],
    hdrs = [
        "track.h",
    ],
    deps = [
        ":sensor",
    ],
)

cc_library(
    name = "track_pool_types",
    srcs = [
        "track_pool_types.cc",
    ],
    hdrs = [
        "track_pool_types.h",
    ],
    deps = [
        ":track",
        "//modules/perception/base:object_pool",
    ],
)

cpplint()
