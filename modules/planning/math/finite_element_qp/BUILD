load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "finite_element_qp",
    deps = [
        ":active_set_augmented_lateral_qp_optimizer",
        ":active_set_lateral_qp_optimizer",
        ":lateral_qp_optimizer",
        ":osqp_lateral_jerk_qp_optimizer",
        ":osqp_lateral_linear_qp_optimizer",
        ":osqp_lateral_qp_optimizer",
    ],
)

cc_library(
    name = "lateral_qp_optimizer",
    srcs = [
        "lateral_qp_optimizer.cc",
    ],
    hdrs = [
        "lateral_qp_optimizer.h",
    ],
    deps = [
        "//cybertron",
        "//modules/planning/lattice/trajectory1d:piecewise_jerk_trajectory1d",
    ],
)

cc_library(
    name = "active_set_lateral_qp_optimizer",
    srcs = [
        "active_set_lateral_qp_optimizer.cc",
    ],
    hdrs = [
        "active_set_lateral_qp_optimizer.h",
    ],
    deps = [
        ":lateral_qp_optimizer",
        "@qpOASES",
    ],
)

cc_library(
    name = "osqp_lateral_qp_optimizer",
    srcs = [
        "osqp_lateral_qp_optimizer.cc",
    ],
    hdrs = [
        "osqp_lateral_qp_optimizer.h",
    ],
    deps = [
        ":lateral_qp_optimizer",
        "@eigen",
        "@osqp",
    ],
)

cc_library(
    name = "osqp_lateral_linear_qp_optimizer",
    srcs = [
        "osqp_lateral_linear_qp_optimizer.cc",
    ],
    hdrs = [
        "osqp_lateral_linear_qp_optimizer.h",
    ],
    deps = [
        ":lateral_qp_optimizer",
        "@eigen",
        "@osqp",
    ],
)

cc_library(
    name = "active_set_augmented_lateral_qp_optimizer",
    srcs = [
        "active_set_augmented_lateral_qp_optimizer.cc",
    ],
    hdrs = [
        "active_set_augmented_lateral_qp_optimizer.h",
    ],
    deps = [
        ":lateral_qp_optimizer",
        "@qpOASES",
    ],
)

cc_library(
    name = "osqp_lateral_jerk_qp_optimizer",
    srcs = [
        "osqp_lateral_jerk_qp_optimizer.cc",
    ],
    hdrs = [
        "osqp_lateral_jerk_qp_optimizer.h",
    ],
    deps = [
        ":lateral_qp_optimizer",
        "@eigen",
        "@osqp",
    ],
)

cpplint()
