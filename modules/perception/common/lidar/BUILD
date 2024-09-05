load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

apollo_cc_library(
    name = "apollo_perception_common_lidar",
    deps = [
        ":apollo_perception_common_lidar_common",
        ":apollo_perception_common_lidar_scene_manager",
    ]
)

apollo_cc_library(
    name = "apollo_perception_common_lidar_common",
    srcs = [
        "common/cloud_mask.cc",
        "common/config_util.cc",
        "common/lidar_frame_pool.cc",
        "common/lidar_object_util.cc",
        "common/object_sequence.cc",
        "common/object_builder.cc",
    ], 
    hdrs = [
        "common/cloud_mask.h",
        "common/config_util.h",
        "common/feature_descriptor.h",
        "common/lidar_error_code.h",
        "common/lidar_frame.h",
        "common/lidar_frame_pool.h",
        "common/lidar_log.h",
        "common/lidar_object_util.h",
        "common/lidar_point_label.h",
        "common/lidar_timer.h",
        "common/object_sequence.h",
        "common/pcl_util.h",
        "common/pointcloud_util.h",
        "common/object_builder.h",
    ],
    deps = [
        "//cyber",
        "//modules/common/util:common_util",
        "//modules/perception/common:perception_common_util",
        "//modules/perception/common:perception_gflags",
        "//modules/perception/common/algorithm:apollo_perception_common_algorithm",
        "//modules/perception/common/base:apollo_perception_common_base",
        "//modules/perception/common/lidar/scene_manager/ground_service/proto:ground_service_config_cc_proto",
        "//modules/perception/common/lidar/scene_manager/proto:scene_manager_config_cc_proto",
        "//modules/perception/common/lidar/scene_manager/roi_service/proto:roi_service_cc_proto",
        "//modules/perception/common/proto:perception_config_schema_cc_proto",
        "@com_google_googletest//:gtest",
        "@eigen",
        "@local_config_pcl//:pcl",
    ]
)

apollo_cc_library(
    name = "apollo_perception_common_lidar_scene_manager",
    srcs = [
        "scene_manager/ground_service/ground_service.cc",
        "scene_manager/roi_service/roi_service.cc",
        "scene_manager/scene_manager.cc",
    ],
    hdrs = [
        "scene_manager/ground_service/ground_service.h",
        "scene_manager/ground_service/ground_struct.h",
        "scene_manager/roi_service/roi_service.h",
        "scene_manager/scene_manager.h",
        "scene_manager/scene_service.h",
    ],
    deps = [
        "//cyber",
        "//modules/common/util:common_util",
        "//modules/perception/common:perception_common_util",
        "//modules/perception/common:perception_gflags",
        "//modules/perception/common/algorithm:apollo_perception_common_algorithm",
        "//modules/perception/common/base:apollo_perception_common_base",
        "//modules/perception/common/lib:apollo_perception_common_lib",
        "//modules/perception/common/lidar/scene_manager/ground_service/proto:ground_service_config_cc_proto",
        "//modules/perception/common/lidar/scene_manager/proto:scene_manager_config_cc_proto",
        "//modules/perception/common/lidar/scene_manager/roi_service/proto:roi_service_cc_proto",
        "//modules/perception/common/proto:perception_config_schema_cc_proto",
        "@com_google_googletest//:gtest",
        "@eigen",
        "@local_config_pcl//:pcl",
    ],
)

apollo_cc_test(
    name = "cloud_mask_test",
    size = "small",
    srcs = ["common/cloud_mask_test.cc"],
    linkstatic = True,
    deps = [
        ":apollo_perception_common_lidar",
        "@com_google_googletest//:gtest_main",
    ],
)

filegroup(
    name = "scene_manager_files",
    srcs = glob([
        "scene_manager/conf/**",
        "scene_manager/data/**",
    ]),
)

cpplint()
apollo_package()