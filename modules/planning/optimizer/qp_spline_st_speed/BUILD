load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "qp_spline_st_boundary_mapper",
    srcs = [
        "qp_spline_st_boundary_mapper.cc",
    ],
    hdrs = [
        "qp_spline_st_boundary_mapper.h",
    ],
    deps = [
        "//modules/common:log",
        "//modules/common/util",
        "//modules/planning/optimizer/st_graph:st_boundary_mapper",
    ],
)

cpplint()
