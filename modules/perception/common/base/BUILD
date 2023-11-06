load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

apollo_cc_library(
    name = "apollo_perception_common_base",
    srcs = [
        "distortion_model.cc",
        "omnidirectional_model.cc",
        "point_cloud_util.cc",
        "polynomial.cc",
        "syncedmem.cc",
        "object_pool_types.cc",
        "blob.cc",
        "object.cc",
        "camera.cc",
    ],
    hdrs = [
        "box.h",
        "comparison_traits.h",
        "common.h",
        "distortion_model.h",
        "frame.h",
        "point_cloud.h",
        "radar_point_cloud.h",
        "frame_supplement.h",
        "object_pool_types.h",
        "image.h",
        "image_8u.h",
        "impending_collision_edge.h",
        "nms.h",
        "omnidirectional_model.h",
        "point.h",
        "point_cloud_util.h",
        "polynomial.h",
        "syncedmem.h",
        "traffic_light.h",
        "test/test_helper.h",
        "concurrent_object_pool.h",
        "light_object_pool.h",
        "object_pool.h",
        "hdmap_struct.h",
        "landmark_struct.h",
        "lane_struct.h",
        "sensor_meta.h",
        "blob.h",
        "object.h",
        "object_supplement.h",
        "object_types.h",
        "vehicle_struct.h",
        "camera.h",
    ],
    deps = [
        "//cyber",
        "//modules/common/util:util_tool",
        "@com_google_googletest//:gtest_main",
        "@eigen",
        "@local_config_cuda//cuda:cuda_headers",
        "@local_config_cuda//cuda:cudart",
        "@opencv//:imgproc",
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
    ],
)

apollo_cc_test(
    name = "blob_test",
    size = "small",
    srcs = ["blob_test.cc"],
    deps = [
        ":apollo_perception_common_base",
        "@com_google_googletest//:gtest_main",
        "@local_config_cuda//cuda:cudart",
    ],
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
        "@local_config_cuda//cuda:cuda_headers",
    ],
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
        "@local_config_cuda//cuda:cudart",
    ],
)

apollo_package()

cpplint()
