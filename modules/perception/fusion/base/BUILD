load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "base",
    deps = [
        ":base_forward_declaration",
        ":base_init_options",
        ":fusion_log",
        ":scene",
        ":sensor",
        ":sensor_data_manager",
        ":track",
        ":track_pool_types",
    ],
)

cc_library(
    name = "base_forward_declaration",
    hdrs = ["base_forward_declaration.h"],
)

cc_library(
    name = "fusion_log",
    hdrs = ["fusion_log.h"],
)

cc_library(
    name = "base_init_options",
    srcs = ["base_init_options.cc"],
    hdrs = ["base_init_options.h"],
    deps = [
        "//modules/perception/lib/config_manager",
    ],
)

cc_library(
    name = "scene",
    srcs = ["scene.cc"],
    hdrs = ["scene.h"],
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
        "//cyber",
        "//modules/perception/base",
        "//modules/perception/base:base_type",
        "//modules/perception/base:frame",
        "//modules/perception/base:object",
        "//modules/perception/base:object_pool_types",
        "//modules/perception/common/sensor_manager",
        "@eigen",
    ],
)

cc_test(
    name = "sensor_frame_test",
    size = "small",
    srcs = ["sensor_frame_test.cc"],
    deps = [
        ":sensor",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "sensor_test",
    size = "small",
    srcs = ["sensor_test.cc"],
    deps = [
        ":sensor",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "sensor_object_test",
    size = "small",
    srcs = ["sensor_object_test.cc"],
    deps = [
        ":sensor",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "sensor_data_manager",
    srcs = ["sensor_data_manager.cc"],
    hdrs = ["sensor_data_manager.h"],
    deps = [
        ":sensor",
        "//modules/perception/base:frame",
        "//modules/perception/common/sensor_manager",
        "//modules/perception/lib/thread",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "sensor_data_manager_test",
    size = "small",
    srcs = ["sensor_data_manager_test.cc"],
    data = [
        "//modules/perception:perception_testdata",
    ],
    deps = [
        ":sensor_data_manager",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "track",
    srcs = ["track.cc"],
    hdrs = ["track.h"],
    deps = [
        ":sensor",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "track_test",
    size = "small",
    srcs = ["track_test.cc"],
    deps = [
        ":track",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "track_pool_types",
    srcs = ["track_pool_types.cc"],
    hdrs = ["track_pool_types.h"],
    deps = [
        ":track",
        "//modules/perception/base:object_pool",
    ],
)

cpplint()
