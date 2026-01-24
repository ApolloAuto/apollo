load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/platform:build_defs.bzl", "if_profiler")

package(default_visibility = ["//visibility:public"])

PERCEPTION_COPTS = ['-DMODULE_NAME=\\"perception\\"']

filegroup(
    name = "lidar_tracking_files",
    srcs = glob([
        "conf/**",
        "dag/**",
        "data/**",
        "launch/**",
    ]),
)

apollo_cc_library(
    name = "apollo_perception_lidar_tracking",
    srcs = [
        "classifier/fused_classifier/ccrf_type_fusion.cc",
        "classifier/fused_classifier/fused_classifier.cc",
        "classifier/fused_classifier/fused_classifier_test.cc",
        "classifier/fused_classifier/util.cc",
        "tracker/association/distance_collection.cc",
        "tracker/association/gnn_bipartite_graph_matcher.cc",
        "tracker/association/multi_hm_bipartite_graph_matcher.cc",
        "tracker/common/mlf_track_data.cc",
        "tracker/common/track_data.cc",
        "tracker/common/track_pool_types.cc",
        "tracker/common/tracked_object.cc",
        "tracker/measurement/measurement_collection.cc",
        "tracker/multi_lidar_fusion/mlf_engine.cc",
        "tracker/multi_lidar_fusion/mlf_motion_filter.cc",
        "tracker/multi_lidar_fusion/mlf_motion_measurement.cc",
        "tracker/multi_lidar_fusion/mlf_motion_refiner.cc",
        "tracker/multi_lidar_fusion/mlf_shape_filter.cc",
        "tracker/multi_lidar_fusion/mlf_track_object_distance.cc",
        "tracker/multi_lidar_fusion/mlf_track_object_matcher.cc",
        "tracker/multi_lidar_fusion/mlf_tracker.cc",
        "tracker/multi_lidar_fusion/mlf_type_filter.cc",
        "tracker/multi_lidar_fusion/util.cc",
        "tracker/multi_lidar_fusion/mlf_direction_filter.cc",
        "tracker/type_fusion/ccrf_type_fusion.cc",
        "tracker/type_fusion/util.cc"
    ],
    hdrs = [
        "classifier/fused_classifier/ccrf_type_fusion.h",
        "classifier/fused_classifier/fused_classifier.h",
        "classifier/fused_classifier/type_fusion_interface.h",
        "classifier/fused_classifier/util.h",
        "interface/base_bipartite_graph_matcher.h",
        "interface/base_classifier.h",
        "interface/base_multi_target_tracker.h",
        "tracker/association/distance_collection.h",
        "tracker/association/gnn_bipartite_graph_matcher.h",
        "tracker/association/multi_hm_bipartite_graph_matcher.h",
        "tracker/common/mlf_track_data.h",
        "tracker/common/track_data.h",
        "tracker/common/track_pool_types.h",
        "tracker/common/tracked_object.h",
        "tracker/measurement/measurement_collection.h",
        "tracker/multi_lidar_fusion/mlf_base_filter.h",
        "tracker/multi_lidar_fusion/mlf_engine.h",
        "tracker/multi_lidar_fusion/mlf_motion_filter.h",
        "tracker/multi_lidar_fusion/mlf_motion_measurement.h",
        "tracker/multi_lidar_fusion/mlf_motion_refiner.h",
        "tracker/multi_lidar_fusion/mlf_shape_filter.h",
        "tracker/multi_lidar_fusion/mlf_track_object_distance.h",
        "tracker/multi_lidar_fusion/mlf_track_object_matcher.h",
        "tracker/multi_lidar_fusion/mlf_tracker.h",
        "tracker/multi_lidar_fusion/mlf_type_filter.h",
        "tracker/multi_lidar_fusion/util.h",
        "tracker/multi_lidar_fusion/mlf_direction_filter.h",
        "tracker/type_fusion/ccrf_type_fusion.h",
        "tracker/type_fusion/type_fusion_interface.h",
        "tracker/type_fusion/util.h"
    ],
    copts = PERCEPTION_COPTS + if_profiler() + ["-DENABLE_PROFILER=1"],
    deps = [
        "//cyber",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/math",
        "//modules/common/util:util_tool",
        "//modules/perception/common:perception_common_util",
        "//modules/perception/common:perception_gflags",
        "//modules/perception/common/algorithm:apollo_perception_common_algorithm",
        "//modules/perception/common/base:apollo_perception_common_base",
        "//modules/perception/common/lib:apollo_perception_common_lib",
        "//modules/perception/common/lidar:apollo_perception_common_lidar",
        "//modules/perception/common/onboard:apollo_perception_common_onboard",
        "//modules/perception/lidar_tracking/classifier/fused_classifier/proto:ccrf_type_fusion_config_cc_proto",
        "//modules/perception/lidar_tracking/classifier/fused_classifier/proto:fused_classifier_config_cc_proto",
        "//modules/perception/lidar_tracking/proto:lidar_tracking_component_config_cc_proto",
        "//modules/perception/lidar_tracking/tracker/multi_lidar_fusion/proto:multi_lidar_fusion_config_cc_proto",
        "//modules/perception/lidar_tracking/tracker/type_fusion/proto:ccrf_type_filter_config_cc_proto",
        "@com_google_googletest//:gtest_main",
        "@eigen",
    ],
)

apollo_component(
    name = "liblidar_tracking_component.so",
    srcs = ["lidar_tracking_component.cc"],
    hdrs = ["lidar_tracking_component.h"],
    copts = PERCEPTION_COPTS + if_profiler() + ["-DENABLE_PROFILER=1"],
    linkopts = ["-lleveldb"],
    deps = [
        "//cyber",
        ":apollo_perception_lidar_tracking",
        "//modules/perception/common/onboard:apollo_perception_common_onboard",
        "//modules/perception/lidar_tracking/proto:lidar_tracking_component_config_cc_proto",
    ],
)

apollo_package()

cpplint()
