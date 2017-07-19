load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "qp_solver",
    srcs = [
        "qp_solver.cc",
    ],
    hdrs = [
        "qp_solver.h",
    ],
    deps = [
        "@eigen//:eigen",
    ],
)

cc_library(
    name = "active_set_qp_solver",
    srcs = [
        "active_set_qp_solver.cc",
    ],
    hdrs = [
        "active_set_qp_solver.h",
    ],
    deps = [
        ":qp_solver",
        "//modules/planning/common:planning_gflags",
        "@eigen//:eigen",
        "//third_party/qp_oases",
    ],
)

cpplint()
