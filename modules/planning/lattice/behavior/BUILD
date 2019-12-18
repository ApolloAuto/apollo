load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "feasible_region",
    srcs = ["feasible_region.cc"],
    hdrs = ["feasible_region.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//cyber/common:log",
        "//modules/common/proto:pnc_point_proto",
        "//modules/planning/common:planning_gflags",
    ],
)

cc_library(
    name = "path_time_graph",
    srcs = ["path_time_graph.cc"],
    hdrs = ["path_time_graph.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//cyber/common:log",
        "//modules/common/math:linear_interpolation",
        "//modules/common/math:path_matcher",
        "//modules/common/proto:pnc_point_proto",
        "//modules/planning/common:frame",
        "//modules/planning/common:obstacle",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/proto:lattice_structure_proto",
        "//modules/planning/reference_line",
    ],
)

cc_library(
    name = "prediction_querier",
    srcs = ["prediction_querier.cc"],
    hdrs = ["prediction_querier.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/common/math",
        "//modules/common/math:path_matcher",
        "//modules/planning/common:obstacle",
        "//modules/planning/proto:lattice_structure_proto",
        "//modules/planning/proto:planning_proto",
    ],
)

cpplint()
