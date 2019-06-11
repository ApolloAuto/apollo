load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "base",
    deps = [
        ":base_type",
        ":blob",
        ":box",
        ":camera",
        ":common",
        ":distortion_model",
        ":frame",
        ":image",
        ":impending_collision_edge",
        ":object",
        ":object_pool",
        ":object_pool_types",
        ":omnidirectional_model",
        ":point_cloud",
        ":point_cloud_util",
        ":polynomial",
        ":syncedmem",
        ":traffic_light",
    ],
)

cc_library(
    name = "base_type",
    hdrs = [
        "hdmap_struct.h",
        "landmark_struct.h",
        "lane_struct.h",
        "sensor_meta.h",
        "vehicle_struct.h",
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
        "//cyber",
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
        "//cyber",
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
    name = "frame",
    hdrs = [
        "frame.h",
        "frame_supplement.h",
    ],
    deps = [
        ":base_type",
        ":blob",
        ":image",
        ":impending_collision_edge",
        "@eigen",
    ],
)

cc_library(
    name = "image",
    hdrs = [
        "image.h",
        "image_8u.h",
    ],
    deps = [
        ":base_type",
        ":blob",
    ],
)

cc_test(
    name = "image_test",
    size = "small",
    deps = [
        ":image",
        "@gtest//:main",
    ],
)

cc_library(
    name = "impending_collision_edge",
    srcs = [
    ],
    hdrs = [
        "impending_collision_edge.h",
    ],
    deps = [
        "//cyber",
        "@eigen",
    ],
)

cc_test(
    name = "impending_collision_edge_test",
    size = "small",
    srcs = [
        "impending_collision_edge_test.cc",
    ],
    deps = [
        ":impending_collision_edge",
        "@gtest//:main",
    ],
)

cc_library(
    name = "object",
    srcs = [
        "object.cc",
    ],
    hdrs = [
        "object.h",
        "object_supplement.h",
        "object_types.h",
        "vehicle_struct.h",
    ],
    deps = [
        ":box",
        ":point_cloud",
        "@eigen",
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
    name = "object_pool",
    hdrs = [
        "concurrent_object_pool.h",
        "light_object_pool.h",
        "object_pool.h",
    ],
    deps = [
        ":base_type",
    ],
)

cc_test(
    name = "object_pool_test",
    size = "small",
    srcs = [
        "object_pool_test.cc",
    ],
    deps = [
        ":frame",
        ":object",
        ":object_pool",
        ":object_pool_types",
        ":point_cloud",
        "//cyber",
        "@eigen",
        "@gtest//:main",
    ],
)

cc_library(
    name = "object_pool_types",
    srcs = [
        "object_pool_types.cc",
    ],
    hdrs = [
        "object_pool_types.h",
    ],
    deps = [
        ":frame",
        ":object",
        ":object_pool",
        ":point_cloud",
        "//cyber",
        "@eigen",
    ],
)

cc_library(
    name = "omnidirectional_model",
    srcs = [
        "omnidirectional_model.cc",
    ],
    hdrs = [
        "omnidirectional_model.h",
    ],
    deps = [
        ":camera",
        ":distortion_model",
        ":polynomial",
        "//cyber",
        "@eigen",
    ],
)

cc_test(
    name = "omnidirectional_model_test",
    size = "small",
    srcs = [
        "omnidirectional_model_test.cc",
    ],
    deps = [
        ":omnidirectional_model",
        "@gtest//:main",
    ],
)

cc_library(
    name = "point_cloud",
    hdrs = [
        "point.h",
        "point_cloud.h",
    ],
    deps = [
        "@eigen",
    ],
)

cc_library(
    name = "point_cloud_util",
    srcs = [
        "point_cloud_util.cc",
    ],
    hdrs = [
        "point_cloud_util.h",
    ],
    linkopts = [
        "-lopencv_core",
        "-lopencv_highgui",
    ],
    deps = [
        ":point_cloud",
        "//cyber",
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
        ":point_cloud",
        "@gtest//:main",
    ],
)

cc_library(
    name = "polynomial",
    srcs = [
        "polynomial.cc",
    ],
    hdrs = [
        "polynomial.h",
    ],
    deps = [
    ],
)

cc_test(
    name = "polynomial_test",
    size = "small",
    srcs = [
        "polynomial_test.cc",
    ],
    deps = [
        ":polynomial",
        "@gtest//:main",
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
        "//cyber",
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

cc_library(
    name = "traffic_light",
    hdrs = [
        "traffic_light.h",
    ],
    deps = [
        "//modules/perception/base:box",
    ],
)

cpplint()
