load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "hm_tracker",
    deps = [
        ":base_filter",
        ":gnn_bipartite_graph_matcher",
        ":hm_multi_target_tracker",
        ":kalman_filter",
        ":measurement_computer",
        ":multi_hm_bipartite_graph_matcher",
        ":object_track_matcher",
        ":track_object_distance",
        ":track_post_processor",
        ":tracker",
    ],
)

cc_library(
    name = "base_filter",
    hdrs = [
        "base_filter.h",
    ],
    deps = [
        "//modules/perception/lib/registerer:registerer",
        "//modules/perception/lidar/lib/tracker/common:track_data",
        "@eigen",
    ],
)

cc_library(
    name = "kalman_filter",
    srcs = [
        "kalman_filter.cc",
    ],
    hdrs = [
        "kalman_filter.h",
    ],
    deps = [
        ":base_filter",
        "//modules/common/util:file_util",
        "//modules/perception/proto:hm_tracker_config_proto",
        "//modules/perception/lib/registerer:registerer",
        "//modules/perception/lib/config_manager:config_manager",
        "//modules/perception/lib/io:file_util",
        "//modules/perception/common/geometry:basic",
        "//modules/perception/lidar/lib/tracker/common:track_data",
        "//framework:cybertron",
        "@eigen",
    ],
)

cc_test(
    name = "kalman_filter_test",
    size = "small",
    srcs = [
        "kalman_filter_test.cc",
    ],
    deps = [
        ":kalman_filter",
        ":hm_tracker",
        "@gtest//:main",
        "//modules/perception/base",
        "//modules/perception/common/io:io_util",
        "//modules/perception/lidar/common",
        "//modules/perception/lidar/lib/interface",
        "//modules/perception/common/graph",
        "//modules/perception/lidar/lib/object_builder",
        "//modules/perception/lidar/lib/tracker/common:track_pool_types",
    ],
)

cc_library(
    name = "measurement_computer",
    srcs = [
        "measurement_computer.cc",
    ],
    hdrs = [
        "measurement_computer.h",
    ],
    deps = [
        "//modules/perception/common/geometry:basic",
        "//modules/perception/common/geometry:common",
        "//modules/perception/lidar/lib/tracker/common:track_data",
        "//modules/perception/lidar/lib/tracker/common:tracked_object",
    ],
)

cc_library(
    name = "track_object_distance",
    srcs = [
        "track_object_distance.cc",
    ],
    hdrs = [
        "track_object_distance.h",
    ],
    deps = [
        "//modules/perception/lidar/lib/tracker/common:track_data",
        "//modules/perception/lidar/lib/tracker/common:tracked_object",
        "//framework:cybertron",
        "@eigen",
    ],
)

cc_library(
    name = "gnn_bipartite_graph_matcher",
    srcs = [
        "gnn_bipartite_graph_matcher.cc",
    ],
    hdrs = [
        "gnn_bipartite_graph_matcher.h",
    ],
    deps = [
        "//modules/perception/common/graph:secure_matrix",
        "//modules/perception/lidar/lib/interface:base_bipartite_graph_matcher",
        "//framework:cybertron",
    ],
)

cc_test(
    name = "gnn_bipartite_graph_matcher_test",
    size = "small",
    srcs = [
        "gnn_bipartite_graph_matcher_test.cc",
    ],
    deps = [
        ":gnn_bipartite_graph_matcher",
        ":hm_tracker",
        "@gtest//:main",
        "//modules/perception/base",
        "//modules/perception/common/io:io_util",
        "//modules/perception/lidar/common",
        "//modules/perception/lidar/lib/interface",
        "//modules/perception/common/graph",
        "//modules/perception/lidar/lib/object_builder",
        "//modules/perception/lidar/lib/tracker/common:track_pool_types",
    ],
)

cc_library(
    name = "track_post_processor",
    srcs = [
        "track_post_processor.cc",
    ],
    hdrs = [
        "track_post_processor.h",
    ],
    deps = [
        "//modules/perception/base:object",
        "//modules/perception/common/geometry:basic",
        "//modules/perception/common/geometry:common",
        "//modules/perception/lidar/lib/tracker/common:track_data",
        "//framework:cybertron",
    ],
)

cc_test(
    name = "track_post_processor_test",
    size = "small",
    srcs = [
        "track_post_processor_test.cc",
    ],
    deps = [
        ":track_post_processor",
        ":hm_tracker",
        "@gtest//:main",
        "//modules/perception/base",
        "//modules/perception/common/io:io_util",
        "//modules/perception/lidar/common",
        "//modules/perception/lidar/lib/interface",
        "//modules/perception/common/graph",
        "//modules/perception/lidar/lib/object_builder",
        "//modules/perception/lidar/lib/tracker/common:track_pool_types",
    ],
)


cc_library(
    name = "multi_hm_bipartite_graph_matcher",
    srcs = [
        "multi_hm_bipartite_graph_matcher.cc",
    ],
    hdrs = [
        "multi_hm_bipartite_graph_matcher.h",
    ],
    deps = [
        "//modules/perception/common/graph:gated_hungarian_bigraph_matcher",
        "//modules/perception/common/graph:secure_matrix",
        "//modules/perception/lidar/lib/interface:base_bipartite_graph_matcher",
        "@eigen",
        "//framework:cybertron",
    ],
)

cc_library(
    name = "object_track_matcher",
    srcs = [
        "object_track_matcher.cc",
    ],
    hdrs = [
        "object_track_matcher.h",
    ],
    deps = [
        ":track_object_distance",
        ":gnn_bipartite_graph_matcher",
        ":multi_hm_bipartite_graph_matcher",
        "//modules/perception/proto:hm_tracker_config_proto",
        "//modules/perception/lib/config_manager:config_manager",
        "//modules/perception/lib/io:file_util",
        "//modules/common/util:file_util",
        "//modules/perception/common/graph:secure_matrix",
        "//modules/perception/lidar/lib/interface:base_bipartite_graph_matcher",
        "//modules/perception/lidar/lib/tracker/common:track_data",
        "//modules/perception/lidar/lib/tracker/common:tracked_object",
        "@eigen",
        "//framework:cybertron",
    ],
)

cc_test(
    name = "object_track_matcher_test",
    size = "small",
    srcs = [
        "object_track_matcher_test.cc",
    ],
    deps = [
        ":object_track_matcher",
        ":hm_tracker",
        "@gtest//:main",
        "//modules/perception/base",
        "//modules/perception/common/io:io_util",
        "//modules/perception/lidar/common",
        "//modules/perception/lidar/lib/interface",
        "//modules/perception/common/graph",
        "//modules/perception/lidar/lib/object_builder",
        "//modules/perception/lidar/lib/tracker/common:track_pool_types",
    ],
)

cc_library(
    name = "tracker",
    srcs = [
        "tracker.cc",
    ],
    hdrs = [
        "tracker.h",
    ],
    deps = [
        ":base_filter",
        ":measurement_computer",
        ":track_post_processor",
        "//modules/common/util:file_util",
        "//modules/perception/proto:hm_tracker_config_proto",
        "//modules/perception/lib/config_manager:config_manager",
        "//modules/perception/lib/io:file_util",
        "//modules/perception/base:object_pool_types",
        "//modules/perception/lidar/lib/tracker/common:track_data",
        "//modules/perception/lidar/lib/tracker/common:track_pool_types",
        "//modules/perception/lidar/lib/tracker/common:tracked_object",
        "//framework:cybertron",
    ],
)

cc_library(
    name = "hm_multi_target_tracker",
    srcs = [
        "hm_multi_target_tracker.cc",
    ],
    hdrs = [
        "hm_multi_target_tracker.h",
    ],
    deps = [
        ":object_track_matcher",
        ":tracker",
        ":track_object_distance",
        "//modules/common/util:file_util",
        "//modules/perception/base:object",
        "//modules/perception/lib/io:file_util",
        "//modules/perception/proto:hm_tracker_config_proto",
        "//modules/perception/lib/config_manager:config_manager",
        "//modules/perception/common/graph:secure_matrix",
        "//modules/perception/common/geometry:convex_hull_2d",
        "//modules/perception/lidar/lib/interface:base_multi_target_tracker",
        "//modules/perception/lidar/lib/interface:base_bipartite_graph_matcher",
        "//modules/perception/lidar/lib/tracker/common:track_data",
        "//modules/perception/lidar/lib/tracker/common:tracked_object",
        "//modules/perception/lidar/lib/tracker/common:track_pool_types",
        "//modules/perception/lidar/common:feature_descriptor",
        "//modules/perception/lidar/common:lidar_object_util",
        "//framework:cybertron",
    ],
)

cc_test(
    name = "hm_multi_target_tracker_test",
    size = "small",
    srcs = [
        "hm_multi_target_tracker_test.cc",
    ],
    deps = [
        ":hm_multi_target_tracker",
        "@gtest//:main",
        "//modules/perception/common/io:io_util",
        "//modules/perception/lidar/lib/object_builder",
    ],
)

cpplint()
