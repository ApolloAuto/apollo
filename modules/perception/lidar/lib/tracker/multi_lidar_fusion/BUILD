load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "mlf_base_filter",
    hdrs = [
        "mlf_base_filter.h",
    ],
    deps = [
        "//modules/perception/lidar/lib/tracker/common:tracked_object",
        "//modules/perception/lidar/lib/tracker/common:mlf_track_data_with_track_pool_types",
        "//modules/perception/lib/registerer:registerer",
    ],
)

cc_library(
    name = "mlf_engine",
    hdrs = [
        "mlf_engine.h",
    ],
    srcs = [
        "mlf_engine.cc",
    ],
    deps = [
        "//modules/perception/lidar/lib/interface:base_multi_target_tracker",
        "//modules/perception/lidar/lib/tracker/multi_lidar_fusion:mlf_tracker",
        "//modules/perception/lidar/lib/tracker/multi_lidar_fusion:mlf_track_object_matcher",
        "//modules/perception/lib/config_manager:config_manager",
        "//modules/perception/lib/io:file_util",
        "//modules/common/util:file_util",
        "//modules/perception/lidar/lib/tracker/multi_lidar_fusion/proto:multi_lidar_fusion_config_proto",
        "//modules/perception/lidar/lib/tracker/common:mlf_track_data_with_track_pool_types",
    ],
)

cc_library(
    name = "mlf_motion_filter",
    srcs = [
        "mlf_motion_filter.cc",
    ],
    hdrs = [
        "mlf_motion_filter.h",
    ],
    deps = [
        "//modules/perception/lidar/lib/tracker/multi_lidar_fusion:mlf_base_filter",
        "//modules/perception/lidar/lib/tracker/multi_lidar_fusion:mlf_motion_refiner",
        "//modules/perception/lidar/lib/tracker/multi_lidar_fusion:mlf_motion_measurement",
        "//modules/perception/common/geometry:basic",
        "//modules/perception/lib/config_manager:config_manager",
        "//modules/perception/lib/io:file_util",
        "//modules/common/util:file_util",
        "//modules/perception/lidar/lib/tracker/multi_lidar_fusion/proto:multi_lidar_fusion_config_proto",
    ],
)


cc_library(
    name = "mlf_motion_measurement",
    srcs = [
        "mlf_motion_measurement.cc",
    ],
    hdrs = [
        "mlf_motion_measurement.h",
    ],
    deps = [
        "//modules/perception/lidar/lib/tracker/common:mlf_track_data_with_track_pool_types",
        "//modules/perception/lidar/lib/tracker/common:tracked_object",
        "//modules/perception/lidar/lib/tracker/measurement:measurement_collection",
    ],
)

cc_library(
    name = "mlf_motion_refiner",
    srcs = [
        "mlf_motion_refiner.cc",
    ],
    hdrs = [
        "mlf_motion_refiner.h",
    ],
    deps = [
        "//modules/perception/lidar/lib/tracker/common:mlf_track_data_with_track_pool_types",
        "//modules/perception/common/geometry:basic",
        "//modules/perception/lib/config_manager:config_manager",
        "//modules/perception/lib/io:file_util",
        "//modules/common/util:file_util",
        "//modules/perception/lidar/lib/tracker/multi_lidar_fusion/proto:multi_lidar_fusion_config_proto",
    ],
)

cc_library(
    name = "mlf_shape_filter",
    srcs = [
        "mlf_shape_filter.cc",
    ],
    hdrs = [
        "mlf_shape_filter.h",
    ],
    deps = [
        "//modules/perception/common/geometry:convex_hull_2d",
        "//modules/perception/lidar/lib/tracker/multi_lidar_fusion:mlf_base_filter",
        "//modules/perception/lib/config_manager:config_manager",
        "//modules/perception/lib/io:file_util",
        "//modules/common/util:file_util",
        "//modules/perception/lidar/lib/tracker/multi_lidar_fusion/proto:multi_lidar_fusion_config_proto",
        "//modules/perception/lidar/common:lidar_object_util",

    ],
)


cc_library(
    name = "mlf_track_object_distance",
    srcs = [
        "mlf_track_object_distance.cc",
    ],
    hdrs = [
        "mlf_track_object_distance.h",
    ],
    deps = [
        "//modules/perception/lidar/lib/tracker/common:tracked_object",
        "//modules/perception/lidar/lib/tracker/common:mlf_track_data_with_track_pool_types",
        "//modules/perception/lib/config_manager:config_manager",
        "//modules/perception/lib/io:file_util",
        "//modules/common/util:file_util",
        "//modules/perception/lidar/lib/tracker/multi_lidar_fusion/proto:multi_lidar_fusion_config_proto",
        "//modules/perception/lidar/lib/tracker/association:distance_collection",
    ],
)

cc_library(
    name = "mlf_track_object_matcher",
    srcs = [
        "mlf_track_object_matcher.cc",
    ],
    hdrs = [
        "mlf_track_object_matcher.h",
    ],
    deps = [
        "//modules/perception/common/graph:secure_matrix",
        "//modules/perception/lidar/lib/interface:base_bipartite_graph_matcher",
        "//modules/perception/lidar/lib/tracker/multi_lidar_fusion:mlf_track_object_distance",
        "//modules/perception/lib/config_manager:config_manager",
        "//modules/perception/lib/io:file_util",
        "//modules/common/util:file_util",
        "//modules/perception/lidar/lib/tracker/multi_lidar_fusion/proto:multi_lidar_fusion_config_proto",
    ],
)

cc_library(
    name = "mlf_tracker",
    srcs = [
        "mlf_tracker.cc",
    ],
    hdrs = [
        "mlf_tracker.h",
    ],
    deps = [
        "//modules/perception/lidar/lib/tracker/common:tracked_object",
        "//modules/perception/lidar/lib/tracker/common:mlf_track_data_with_track_pool_types",
        "//modules/perception/lidar/lib/tracker/multi_lidar_fusion:mlf_base_filter",
        "//modules/perception/lib/config_manager:config_manager",
        "//modules/perception/lib/io:file_util",
        "//modules/common/util:file_util",
        "//modules/perception/lidar/lib/tracker/multi_lidar_fusion/proto:multi_lidar_fusion_config_proto",
    ],
)


cpplint()
