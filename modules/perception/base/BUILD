load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

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

cpplint()
