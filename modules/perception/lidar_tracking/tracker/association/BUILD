load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "multi_hm_bipartite_graph_matcher",
    srcs = ["multi_hm_bipartite_graph_matcher.cc"],
    hdrs = ["multi_hm_bipartite_graph_matcher.h"],
    deps = [
        "//cyber",
        "//modules/perception/common/algorithm/graph:gated_hungarian_bigraph_matcher",
        "//modules/perception/common/algorithm/graph:secure_matrix",
        "//modules/perception/lidar_tracking/interface:base_bipartite_graph_matcher",
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
        "//modules/perception/common/algorithm/graph:gated_hungarian_bigraph_matcher",
        "//modules/perception/common/algorithm/graph:secure_matrix",
        "//modules/perception/lidar_tracking/interface:base_bipartite_graph_matcher",
    ],
    alwayslink = True,
)

cc_library(
    name = "distance_collection",
    srcs = ["distance_collection.cc"],
    hdrs = ["distance_collection.h"],
    deps = [
        "//modules/common/math",
        "//modules/perception/common/algorithm/geometry:basic",
        "//modules/perception/common/algorithm/geometry:common",
        "//modules/perception/common/lidar/common:lidar_log",
        "//modules/perception/lidar_tracking/tracker/common:mlf_track_data_with_track_pool_types",
        "//modules/perception/lidar_tracking/tracker/common:track_data",
        "//modules/perception/lidar_tracking/tracker/common:tracked_object",
    ],
    alwayslink = True,
)

apollo_package()
cpplint()
