load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "multi_hm_bipartite_graph_matcher",
    srcs = ["multi_hm_bipartite_graph_matcher.cc"],
    hdrs = ["multi_hm_bipartite_graph_matcher.h"],
    deps = [
        "//cyber",
        "//modules/perception/common/graph:gated_hungarian_bigraph_matcher",
        "//modules/perception/common/graph:secure_matrix",
        "//modules/perception/lidar/lib/interface:base_bipartite_graph_matcher",
        "@eigen",
    ],
    alwayslink = True,
)

cc_library(
    name = "gnn_bipartite_graph_matcher",
    srcs = ["gnn_bipartite_graph_matcher.cc"],
    hdrs = ["gnn_bipartite_graph_matcher.h"],
    deps = [
        "//cyber",
        "//modules/perception/common/graph:gated_hungarian_bigraph_matcher",
        "//modules/perception/common/graph:secure_matrix",
        "//modules/perception/lidar/lib/interface:base_bipartite_graph_matcher",
    ],
    alwayslink = True,
)

cc_library(
    name = "distance_collection",
    srcs = ["distance_collection.cc"],
    hdrs = ["distance_collection.h"],
    deps = [
        "//modules/perception/common/geometry:basic",
        "//modules/perception/common/geometry:common",
        "//modules/perception/lidar/common:lidar_log",
        "//modules/perception/lidar/lib/tracker/common:track_data",
        "//modules/perception/lidar/lib/tracker/common:tracked_object",
    ],
    alwayslink = True,
)

cpplint()
