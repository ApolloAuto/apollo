load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "mlf_base_filter",
    hdrs = ["mlf_base_filter.h"],
    deps = [
        "//modules/perception/common/lib/registerer",
        "//modules/perception/lidar_tracking/tracker/common:mlf_track_data_with_track_pool_types",
        "//modules/perception/lidar_tracking/tracker/common:tracked_object",
        "//modules/perception/common/lib/interface:base_init_options",
    ],
)

cc_library(
    name = "mlf_engine",
    srcs = ["mlf_engine.cc"],
    hdrs = ["mlf_engine.h"],
    deps = [
        "//cyber/common:file",
        "//modules/perception/lidar_tracking/interface:base_multi_target_tracker",
        "//modules/perception/lidar_tracking/tracker/common:mlf_track_data_with_track_pool_types",
        "//modules/perception/lidar_tracking/tracker/multi_lidar_fusion:mlf_track_object_matcher",
        "//modules/perception/lidar_tracking/tracker/multi_lidar_fusion:mlf_tracker",
        "//modules/perception/lidar_tracking/tracker/semantic_map:perception_evaluator_manager",
        "//modules/perception/common:util",
        "//modules/perception/common/onboard/msg_serializer",
        "//modules/prediction/container/obstacles:obstacles_container",
        "//modules/prediction/container/pose:pose_container",
    ],
    alwayslink = True,
)

cc_library(
    name = "mlf_motion_filter",
    srcs = ["mlf_motion_filter.cc"],
    hdrs = ["mlf_motion_filter.h"],
    deps = [
        "//cyber/common:file",
        "//modules/perception/common:util",
        "//modules/perception/common/algorithm/geometry:basic",
        "//modules/perception/lidar_tracking/tracker/multi_lidar_fusion:mlf_base_filter",
        "//modules/perception/lidar_tracking/tracker/multi_lidar_fusion:mlf_motion_measurement",
        "//modules/perception/lidar_tracking/tracker/multi_lidar_fusion:mlf_motion_refiner",
    ],
    alwayslink = True,
)

cc_library(
    name = "mlf_motion_measurement",
    srcs = ["mlf_motion_measurement.cc"],
    hdrs = ["mlf_motion_measurement.h"],
    deps = [
        "//modules/perception/lidar_tracking/tracker/common:mlf_track_data_with_track_pool_types",
        "//modules/perception/lidar_tracking/tracker/common:tracked_object",
        "//modules/perception/lidar_tracking/tracker/measurement:measurement_collection",
    ],
    alwayslink = True,
)

cc_library(
    name = "mlf_motion_refiner",
    srcs = ["mlf_motion_refiner.cc"],
    hdrs = ["mlf_motion_refiner.h"],
    deps = [
        "//cyber/common:file",
        "//modules/perception/common:util",
        "//modules/perception/common/algorithm/geometry:basic",
        "//modules/perception/lidar_tracking/tracker/common:mlf_track_data_with_track_pool_types",
        "//modules/perception/lidar_tracking/tracker/multi_lidar_fusion/proto:multi_lidar_fusion_config_cc_proto",
        "//modules/perception/common/lib/interface:base_init_options",
    ],
    alwayslink = True,
)

cc_library(
    name = "mlf_shape_filter",
    srcs = ["mlf_shape_filter.cc"],
    hdrs = ["mlf_shape_filter.h"],
    deps = [
        "//cyber/common:file",
        "//modules/perception/common:util",
        "//modules/perception/common/algorithm/geometry:convex_hull_2d",
        "//modules/perception/common/lidar/common:lidar_object_util",
        "//modules/perception/lidar_tracking/tracker/multi_lidar_fusion:mlf_base_filter",
        "//modules/perception/lidar_tracking/tracker/multi_lidar_fusion/proto:multi_lidar_fusion_config_cc_proto",
    ],
    alwayslink = True,
)

cc_library(
    name = "mlf_track_object_distance",
    srcs = ["mlf_track_object_distance.cc"],
    hdrs = ["mlf_track_object_distance.h"],
    deps = [
        "//cyber/common:file",
        "//modules/perception/common:util",
        "//modules/perception/lidar_tracking/tracker/association:distance_collection",
        "//modules/perception/lidar_tracking/tracker/common:mlf_track_data_with_track_pool_types",
        "//modules/perception/lidar_tracking/tracker/common:tracked_object",
        "//modules/perception/lidar_tracking/tracker/multi_lidar_fusion/proto:multi_lidar_fusion_config_cc_proto",
        "//modules/perception/common/lib/interface:base_init_options",
    ],
    alwayslink = True,
)

cc_library(
    name = "mlf_track_object_matcher",
    srcs = ["mlf_track_object_matcher.cc"],
    hdrs = ["mlf_track_object_matcher.h"],
    deps = [
        "//cyber/common:file",
        "//modules/perception/common/algorithm/graph:secure_matrix",
        "//modules/perception/lidar_tracking/interface:base_bipartite_graph_matcher",
        "//modules/perception/lidar_tracking/tracker/association:distance_collection",
        "//modules/perception/lidar_tracking/tracker/association:gnn_bipartite_graph_matcher",
        "//modules/perception/lidar_tracking/tracker/association:multi_hm_bipartite_graph_matcher",
        "//modules/perception/lidar_tracking/tracker/multi_lidar_fusion:mlf_track_object_distance",
        "//modules/perception/common/lib/interface:base_init_options",
    ],
    alwayslink = True,
)

cc_library(
    name = "mlf_tracker",
    srcs = ["mlf_tracker.cc"],
    hdrs = ["mlf_tracker.h"],
    deps = [
        "//cyber/common:file",
        "//modules/perception/common:util",
        "//modules/perception/lidar_tracking/tracker/common:mlf_track_data_with_track_pool_types",
        "//modules/perception/lidar_tracking/tracker/common:tracked_object",
        "//modules/perception/lidar_tracking/tracker/multi_lidar_fusion:mlf_base_filter",
        "//modules/perception/lidar_tracking/tracker/multi_lidar_fusion:mlf_shape_filter",
        "//modules/perception/lidar_tracking/tracker/multi_lidar_fusion:mlf_motion_filter",
        "//modules/perception/lidar_tracking/tracker/multi_lidar_fusion/proto:multi_lidar_fusion_config_cc_proto",
        "//modules/perception/common/lib/interface:base_init_options",
    ],
    alwayslink = True,
)

apollo_package()
cpplint()
