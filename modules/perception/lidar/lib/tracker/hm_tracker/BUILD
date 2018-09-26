load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

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

cpplint()
