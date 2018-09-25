load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "base_type",
    hdrs = [
        "sensor_meta.h",
    ],
    deps = [
    ],
)

cc_library(
    name = "blob",
    srcs = [
        "blob.cc",
    ],
    hdrs = [
        "blob.h",
    ],
    deps = [
        ":common",
        ":syncedmem",
    ],
)

cc_test(
    name = "blob_test",
    size = "small",
    srcs = [
        "blob_test.cc",
    ],
    deps = [
        ":blob",
        "//modules/perception/base/test:test_helper",
        "@cuda",
        "@gtest//:main",
    ],
)

cc_library(
    name = "box",
    hdrs = [
        "box.h",
        "comparison_traits.h",
    ],
    deps = [
        ":point_cloud",
    ],
)

cc_test(
    name = "box_test",
    size = "small",
    srcs = [
        "box_test.cc",
    ],
    deps = [
        ":box",
        "@gtest//:main",
    ],
)

cc_library(
    name = "camera",
    srcs = [
        "camera.cc",
    ],
    hdrs = [
        "camera.h",
    ],
    deps = [
        "@eigen",
    ],
)

cc_test(
    name = "camera_test",
    size = "small",
    srcs = [
        "camera_test.cc",
    ],
    deps = [
        ":camera",
        "@gtest//:main",
    ],
)

cc_library(
    name = "common",
    hdrs = [
        "common.h",
    ],
    deps = [
        "@cuda",
    ],
)

cc_test(
    name = "common_test",
    size = "small",
    srcs = [
        "common_test.cc",
    ],
    deps = [
        ":common",
        "@gtest//:main",
    ],
)

cc_library(
    name = "distortion_model",
    srcs = [
        "distortion_model.cc",
    ],
    hdrs = [
        "distortion_model.h",
    ],
    deps = [
        ":camera",
        "//framework:cybertron",
        "@eigen",
    ],
)

cc_test(
    name = "distortion_model_test",
    size = "small",
    srcs = [
        "distortion_model_test.cc",
    ],
    deps = [
        ":distortion_model",
        "@gtest//:main",
    ],
)

cc_library(
    name = "object",
    hdrs = [
        "object.h",
        "object_supplement.h",
        "object_types.h",
        "vehicle_struct.h",
    ],
    srcs = [
        "object.cc",
    ],
    deps = [
        ":point_cloud",
        ":box",
    ],
)

cc_test(
    name = "object_test",
    size = "small",
    srcs = [
        "object_test.cc",
    ],
    deps = [
        ":object",
        "@gtest//:main",
    ],
)

cc_library(
    name = "point_cloud",
    hdrs = [
        "point.h",
        "point_cloud.h",
        "point_cloud_types.h",
    ],
    deps = [
        "@eigen",
    ],
)

cc_test(
    name = "point_cloud_test",
    size = "small",
    srcs = [
        "point_cloud_test.cc",
    ],
    deps = [
        "@gtest//:main",
        ":point_cloud",
    ],
)

cc_library(
    name = "syncedmem",
    srcs = [
        "syncedmem.cc",
    ],
    hdrs = [
        "syncedmem.h",
    ],
    deps = [
        ":common",
        "//framework:cybertron",
    ],
)

cc_test(
    name = "syncedmem_test",
    size = "small",
    srcs = [
        "syncedmem_test.cc",
    ],
    deps = [
        ":syncedmem",
        "//modules/perception/base/test:test_helper",
        "@cuda",
        "@gtest//:main",
    ],
)

cpplint()
