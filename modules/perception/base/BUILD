load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//third_party/gpus:common.bzl", "if_cuda", "if_rocm")

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
    deps = [
        "//modules/common/util:util_tool",
    ],
)

cc_library(
    name = "blob",
    srcs = ["blob.cc"],
    hdrs = ["blob.h"],
    deps = [
        ":common",
        ":syncedmem",
        "//cyber",
        "@com_google_absl//:absl",
    ] + if_cuda([
        "@local_config_cuda//cuda:cuda_headers",
    ]) + if_rocm([
        "@local_config_rocm//rocm:rocm_headers",
    ]),
)

cc_test(
    name = "blob_test",
    size = "small",
    srcs = ["blob_test.cc"],
    deps = [
        ":blob",
        "//modules/perception/base/test:test_helper",
        "@com_google_googletest//:gtest_main",
    ] + if_cuda([
        "@local_config_cuda//cuda:cudart",
    ]) + if_rocm([
        "@local_config_rocm//rocm:hip",
    ]),
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
    srcs = ["box_test.cc"],
    deps = [
        ":box",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

cc_library(
    name = "camera",
    srcs = ["camera.cc"],
    hdrs = ["camera.h"],
    deps = [
        "@eigen",
    ],
)

cc_test(
    name = "camera_test",
    size = "small",
    srcs = ["camera_test.cc"],
    linkopts = ["-lm"],
    deps = [
        ":camera",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "common",
    hdrs = ["common.h"],
    deps = [
    ] + if_cuda([
        "@local_config_cuda//cuda:cudart",
    ]) + if_rocm([
        "@local_config_rocm//rocm:hip",
    ]),
)

cc_test(
    name = "common_test",
    size = "small",
    srcs = ["common_test.cc"],
    deps = [
        ":common",
        "@com_google_googletest//:gtest_main",
    ] + if_cuda([
        "@local_config_cuda//cuda:cuda_headers",
    ]) + if_rocm([
        "@local_config_rocm//rocm:rocm_headers",
    ]),
)

cc_library(
    name = "distortion_model",
    srcs = ["distortion_model.cc"],
    hdrs = ["distortion_model.h"],
    deps = [
        ":camera",
        "//cyber",
        "@eigen",
    ],
)

cc_test(
    name = "distortion_model_test",
    size = "small",
    srcs = ["distortion_model_test.cc"],
    deps = [
        ":distortion_model",
        "@com_google_googletest//:gtest_main",
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
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

cc_library(
    name = "impending_collision_edge",
    hdrs = ["impending_collision_edge.h"],
    deps = [
        "//cyber",
        "@eigen",
    ],
)

cc_test(
    name = "impending_collision_edge_test",
    size = "small",
    srcs = ["impending_collision_edge_test.cc"],
    deps = [
        ":impending_collision_edge",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "object",
    srcs = ["object.cc"],
    hdrs = [
        "object.h",
        "object_supplement.h",
        "object_types.h",
        "vehicle_struct.h",
    ],
    deps = [
        ":box",
        ":point_cloud",
        "//modules/common_msgs/basic_msgs:error_code_cc_proto",
        "//modules/common_msgs/basic_msgs:geometry_cc_proto",
        "//modules/common_msgs/basic_msgs:header_cc_proto",
        "//modules/common_msgs/basic_msgs:pnc_point_cc_proto",
        "//modules/common_msgs/map_msgs:map_geometry_cc_proto",
        "//modules/common_msgs/map_msgs:map_id_cc_proto",
        "//modules/common_msgs/map_msgs:map_lane_cc_proto",
        "//modules/common_msgs/perception_msgs:perception_obstacle_cc_proto",
        "//modules/common_msgs/prediction_msgs:feature_cc_proto",
        "//modules/common_msgs/prediction_msgs:lane_graph_cc_proto",
        "//modules/common_msgs/prediction_msgs:prediction_point_cc_proto",
        "//modules/common/util",
        "@boost",
        "@eigen",
    ],
)

cc_test(
    name = "object_test",
    size = "small",
    srcs = ["object_test.cc"],
    deps = [
        ":object",
        "@com_google_googletest//:gtest_main",
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
    srcs = ["object_pool_test.cc"],
    deps = [
        ":frame",
        ":object",
        ":object_pool",
        ":object_pool_types",
        ":point_cloud",
        "//cyber",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "object_pool_types",
    srcs = ["object_pool_types.cc"],
    hdrs = ["object_pool_types.h"],
    deps = [
        ":frame",
        ":object",
        ":object_pool",
        ":point_cloud",
        "//cyber",
    ],
)

cc_library(
    name = "omnidirectional_model",
    srcs = ["omnidirectional_model.cc"],
    hdrs = ["omnidirectional_model.h"],
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
    srcs = ["omnidirectional_model_test.cc"],
    deps = [
        ":omnidirectional_model",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "point",
    hdrs = ["point.h"],
)

cc_library(
    name = "point_cloud",
    hdrs = ["point_cloud.h"],
    deps = [
        ":point",
        "//modules/common/util:util_tool",
        "@eigen",
    ],
)

cc_library(
    name = "point_cloud_util",
    srcs = ["point_cloud_util.cc"],
    hdrs = ["point_cloud_util.h"],
    deps = [
        ":point_cloud",
        "//cyber",
        "@eigen",
        "@opencv//:imgproc",
    ],
)

cc_test(
    name = "point_cloud_test",
    size = "small",
    srcs = ["point_cloud_test.cc"],
    linkopts = ["-lm"],
    deps = [
        ":point_cloud",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "polynomial",
    srcs = ["polynomial.cc"],
    hdrs = ["polynomial.h"],
)

cc_test(
    name = "polynomial_test",
    size = "small",
    srcs = ["polynomial_test.cc"],
    deps = [
        ":polynomial",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "syncedmem",
    srcs = ["syncedmem.cc"],
    hdrs = ["syncedmem.h"],
    deps = [
        ":common",
        "//cyber",
    ] + if_cuda([
        "@local_config_cuda//cuda:cuda_headers",
    ]) + if_rocm([
        "@local_config_rocm//rocm:rocm_headers",
    ]),
)

cc_test(
    name = "syncedmem_test",
    size = "small",
    srcs = ["syncedmem_test.cc"],
    deps = [
        ":syncedmem",
        "//modules/perception/base/test:test_helper",
        "@com_google_googletest//:gtest_main",
    ] + if_cuda([
        "@local_config_cuda//cuda:cudart",
    ]) + if_rocm([
        "@local_config_rocm//rocm:hip",
    ]),
)

cc_library(
    name = "traffic_light",
    hdrs = ["traffic_light.h"],
    deps = [
        "//modules/perception/base:box",
    ],
)

cpplint()
