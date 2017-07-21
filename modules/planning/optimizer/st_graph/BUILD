package(default_visibility = ["//visibility:public"])

cc_library(
    name = "st_graph_point",
    srcs = [
        "st_graph_point.cc",
    ],
    hdrs = [
        "st_graph_point.h",
    ],
    deps = [
        "//modules/common/proto:path_point_proto",
        "//modules/planning/common:planning_gflags",
    ],
)

cc_library(
    name = "st_graph_boundary",
    srcs = [
        "st_graph_boundary.cc",
    ],
    hdrs = [
        "st_graph_boundary.h",
    ],
    deps = [
        "//modules/common:log",
        "//modules/common/math:math",
        "//modules/planning/proto:planning_proto",
        ":st_graph_point",
    ],
)

cc_library(
    name = "st_graph_data",
    srcs = [
        "st_graph_data.cc",
    ],
    hdrs = [
        "st_graph_data.h",
    ],
    deps = [
        ":st_graph_boundary",
        "//modules/common/proto:path_point_proto",
        "//modules/planning/common:planning_gflags",
    ],
)


