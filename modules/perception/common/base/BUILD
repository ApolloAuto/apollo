load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")
load("//third_party/gpus:common.bzl", "gpu_library", "if_cuda", "if_rocm")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

apollo_cc_library(
    name = "apollo_perception_common_base",
    srcs = [
        "blob.cc",
        "camera.cc",
        "distortion_model.cc",
        "object.cc",
        "object_pool_types.cc",
        "omnidirectional_model.cc",
        "point_cloud_util.cc",
        "polynomial.cc",
        "syncedmem.cc",
    ],
    hdrs = [
        "blob.h",
        "box.h",
        "camera.h",
        "common.h",
        "comparison_traits.h",
        "concurrent_object_pool.h",
        "distortion_model.h",
        "frame.h",
        "frame_supplement.h",
        "hdmap_struct.h",
        "image.h",
        "image_8u.h",
        "impending_collision_edge.h",
        "landmark_struct.h",
        "lane_struct.h",
        "light_object_pool.h",
        "nms.h",
        "object.h",
        "object_pool.h",
        "object_pool_types.h",
        "object_supplement.h",
        "object_types.h",
        "omnidirectional_model.h",
        "point.h",
        "point_cloud.h",
        "point_cloud_util.h",
        "polynomial.h",
        "radar_point_cloud.h",
        "sensor_meta.h",
        "syncedmem.h",
        "test/test_helper.h",
        "traffic_light.h",
        "vehicle_struct.h",
    ],
    deps = [
        "//cyber",
        "//modules/common/util:util_tool",
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
        "@boost",
        "@com_google_googletest//:gtest_main",
        "@eigen",
        "@opencv//:imgproc",
    ] + if_cuda([
        "@local_config_cuda//cuda:cuda_headers",
        "@local_config_cuda//cuda:cudart",
    ]) + if_rocm([
        "@local_config_rocm//rocm:rocm_headers",
        "@local_config_rocm//rocm:hip",
    ]),
)

apollo_cc_test(
    name = "blob_test",
    size = "small",
    srcs = ["blob_test.cc"],
    deps = [
        ":apollo_perception_common_base",
        "@com_google_googletest//:gtest_main",
    ] + if_cuda([
        "@local_config_cuda//cuda:cudart",
    ]) + if_rocm([
        "@local_config_rocm//rocm:hip",
    ]),
)

apollo_cc_test(
    name = "box_test",
    size = "small",
    srcs = ["box_test.cc"],
    linkstatic = True,
    deps = [
        ":apollo_perception_common_base",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "camera_test",
    size = "small",
    srcs = ["camera_test.cc"],
    linkopts = ["-lm"],
    deps = [
        ":apollo_perception_common_base",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "common_test",
    size = "small",
    srcs = ["common_test.cc"],
    deps = [
        ":apollo_perception_common_base",
        "@com_google_googletest//:gtest_main",
    ] + if_cuda([
        "@local_config_cuda//cuda:cuda_headers",
    ]) + if_rocm([
        "@local_config_rocm//rocm:rocm_headers",
    ]),
)

apollo_cc_test(
    name = "distortion_model_test",
    size = "small",
    srcs = ["distortion_model_test.cc"],
    deps = [
        ":apollo_perception_common_base",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "image_test",
    size = "small",
    linkstatic = True,
    deps = [
        ":apollo_perception_common_base",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "impending_collision_edge_test",
    size = "small",
    srcs = ["impending_collision_edge_test.cc"],
    deps = [
        ":apollo_perception_common_base",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "object_test",
    size = "small",
    srcs = ["object_test.cc"],
    deps = [
        ":apollo_perception_common_base",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "object_pool_test",
    size = "small",
    srcs = ["object_pool_test.cc"],
    deps = [
        ":apollo_perception_common_base",
        "//cyber",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "omnidirectional_model_test",
    size = "small",
    srcs = ["omnidirectional_model_test.cc"],
    deps = [
        ":apollo_perception_common_base",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "point_cloud_test",
    size = "small",
    srcs = ["point_cloud_test.cc"],
    linkopts = ["-lm"],
    deps = [
        ":apollo_perception_common_base",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "polynomial_test",
    size = "small",
    srcs = ["polynomial_test.cc"],
    deps = [
        ":apollo_perception_common_base",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "syncedmem_test",
    size = "small",
    srcs = ["syncedmem_test.cc"],
    deps = [
        ":apollo_perception_common_base",
        "@com_google_googletest//:gtest_main",
    ] + if_cuda([
        "@local_config_cuda//cuda:cudart",
    ]) + if_rocm([
        "@local_config_rocm//rocm:hip",
    ]),
)

apollo_package()

cpplint()
