load("@rules_cc//cc:defs.bzl", "cc_library", "cc_binary")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "mlf_base_filter",
    hdrs = ["mlf_base_filter.h"],
    deps = [
        "//modules/perception/lib/registerer",
        "//modules/perception/lidar/lib/tracker/common:mlf_track_data_with_track_pool_types",
        "//modules/perception/lidar/lib/tracker/common:tracked_object",
    ],
)

cc_library(
    name = "mlf_engine",
    srcs = ["mlf_engine.cc"],
    hdrs = ["mlf_engine.h"],
    deps = [
        "//cyber",
        "//modules/perception/lib/config_manager",
        "//modules/perception/lidar/lib/interface:base_multi_target_tracker",
        "//modules/perception/lidar/lib/tracker/common:mlf_track_data_with_track_pool_types",
        "//modules/perception/lidar/lib/tracker/multi_lidar_fusion:mlf_track_object_matcher",
        "//modules/perception/lidar/lib/tracker/multi_lidar_fusion:mlf_tracker",
        "//modules/perception/onboard/msg_serializer",
        "//modules/perception/pipeline:plugin_factory",
        "//modules/perception/pipeline:stage",
        "//modules/perception/pipeline/proto/stage:mlf_engine_config_cc_proto",
        ":external_mlf_lib"
    ],
    alwayslink = True,
)

install(
    name = "install",
    library_dest = "perception/lib",
    targets = [
        ":libexternal_mlf_lib.so",
    ],
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "libexternal_mlf_lib.so",
    linkstatic = True,
    linkshared = True,
    deps = [
        "//modules/prediction/container/obstacles:obstacles_container",
        "//modules/prediction/container/pose:pose_container",
    ]
)

cc_library(
    name = "external_mlf_lib",
    srcs = [":libexternal_mlf_lib.so"],
    deps = [
        "//modules/prediction/container/obstacles:obstacles_container",
        "//modules/prediction/container/pose:pose_container",
    ]
)

cc_library(
    name = "mlf_motion_filter",
    srcs = ["mlf_motion_filter.cc"],
    hdrs = ["mlf_motion_filter.h"],
    deps = [
        "//cyber",
        "//modules/perception/common/geometry:basic",
        "//modules/perception/lib/config_manager",
        "//modules/perception/lidar/lib/tracker/multi_lidar_fusion:mlf_base_filter",
        "//modules/perception/lidar/lib/tracker/multi_lidar_fusion:mlf_motion_measurement",
        "//modules/perception/lidar/lib/tracker/multi_lidar_fusion:mlf_motion_refiner",
    ],
    alwayslink = True,
)

cc_library(
    name = "mlf_motion_measurement",
    srcs = ["mlf_motion_measurement.cc"],
    hdrs = ["mlf_motion_measurement.h"],
    deps = [
        "//modules/perception/lidar/lib/tracker/common:mlf_track_data_with_track_pool_types",
        "//modules/perception/lidar/lib/tracker/common:tracked_object",
        "//modules/perception/lidar/lib/tracker/measurement:measurement_collection",
    ],
    alwayslink = True,
)

cc_library(
    name = "mlf_motion_refiner",
    srcs = ["mlf_motion_refiner.cc"],
    hdrs = ["mlf_motion_refiner.h"],
    deps = [
        "//cyber",
        "//modules/perception/common/geometry:basic",
        "//modules/perception/lib/config_manager",
        "//modules/perception/lidar/lib/tracker/common:mlf_track_data_with_track_pool_types",
        "//modules/perception/pipeline/proto/plugin:multi_lidar_fusion_config_cc_proto",
    ],
    alwayslink = True,
)

cc_library(
    name = "mlf_shape_filter",
    srcs = ["mlf_shape_filter.cc"],
    hdrs = ["mlf_shape_filter.h"],
    deps = [
        "//cyber",
        "//modules/perception/common/geometry:convex_hull_2d",
        "//modules/perception/lib/config_manager",
        "//modules/perception/lidar/common:lidar_object_util",
        "//modules/perception/lidar/lib/tracker/multi_lidar_fusion:mlf_base_filter",
        "//modules/perception/pipeline/proto/plugin:multi_lidar_fusion_config_cc_proto",
    ],
    alwayslink = True,
)

cc_library(
    name = "mlf_track_object_distance",
    srcs = ["mlf_track_object_distance.cc"],
    hdrs = ["mlf_track_object_distance.h"],
    deps = [
        "//cyber",
        "//modules/perception/lib/config_manager",
        "//modules/perception/lidar/lib/tracker/association:distance_collection",
        "//modules/perception/lidar/lib/tracker/common:mlf_track_data_with_track_pool_types",
        "//modules/perception/lidar/lib/tracker/common:tracked_object",
        "//modules/perception/pipeline/proto/plugin:multi_lidar_fusion_config_cc_proto",
    ],
    alwayslink = True,
)

cc_library(
    name = "mlf_track_object_matcher",
    srcs = ["mlf_track_object_matcher.cc"],
    hdrs = ["mlf_track_object_matcher.h"],
    deps = [
        "//cyber",
        "//modules/perception/common/graph:secure_matrix",
        "//modules/perception/lib/config_manager",
        "//modules/perception/lidar/lib/interface:base_bipartite_graph_matcher",
        "//modules/perception/lidar/lib/tracker/association:distance_collection",
        "//modules/perception/lidar/lib/tracker/association:gnn_bipartite_graph_matcher",
        "//modules/perception/lidar/lib/tracker/association:multi_hm_bipartite_graph_matcher",
        "//modules/perception/lidar/lib/tracker/multi_lidar_fusion:mlf_track_object_distance",
        "//modules/perception/pipeline:plugin",
    ],
    alwayslink = True,
)

cc_library(
    name = "mlf_tracker",
    srcs = ["mlf_tracker.cc"],
    hdrs = ["mlf_tracker.h"],
    deps = [
        "//cyber",
        "//modules/perception/lib/config_manager",
        "//modules/perception/lidar/lib/tracker/common:mlf_track_data_with_track_pool_types",
        "//modules/perception/lidar/lib/tracker/common:tracked_object",
        "//modules/perception/lidar/lib/tracker/multi_lidar_fusion:mlf_base_filter",
        "//modules/perception/lidar/lib/tracker/multi_lidar_fusion:mlf_motion_filter",
        "//modules/perception/lidar/lib/tracker/multi_lidar_fusion:mlf_motion_measurement",
        "//modules/perception/lidar/lib/tracker/multi_lidar_fusion:mlf_motion_refiner",
        "//modules/perception/lidar/lib/tracker/multi_lidar_fusion:mlf_shape_filter",
        "//modules/perception/pipeline:plugin",
    ],
    alwayslink = True,
)

cpplint()
