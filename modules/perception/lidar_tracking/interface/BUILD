load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "base_bipartite_graph_matcher",
    hdrs = ["base_bipartite_graph_matcher.h"],
    deps = [
        "//modules/perception/common/algorithm/graph:secure_matrix",
        "//modules/perception/common/lib/registerer",
        "//modules/perception/common/lib/interface:base_init_options",
        "@eigen",
    ],
)

cc_library(
    name = "base_classifier",
    hdrs = ["base_classifier.h"],
    deps = [
        "//modules/perception/common/lib/registerer",
        "//modules/perception/common/lidar/common:lidar_frame",
        "//modules/perception/common/lib/interface:base_init_options",
    ],
)

cc_library(
    name = "base_multi_target_tracker",
    hdrs = ["base_multi_target_tracker.h"],
    deps = [
        "//modules/perception/common/lib/registerer",
        "//modules/perception/common/lidar/common:lidar_frame",
        "//modules/perception/common/lib/interface:base_init_options",
    ],
)

apollo_package()
cpplint()
