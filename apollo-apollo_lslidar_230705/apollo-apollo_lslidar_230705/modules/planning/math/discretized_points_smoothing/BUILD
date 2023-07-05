load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "cos_theta_smoother",
    srcs = ["cos_theta_smoother.cc"],
    hdrs = ["cos_theta_smoother.h"],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    deps = [
        ":cos_theta_ipopt_interface",
        "//cyber",
        "//modules/planning/proto/math:cos_theta_smoother_config_cc_proto",
        "@ipopt",
    ],
)

cc_library(
    name = "cos_theta_ipopt_interface",
    srcs = ["cos_theta_ipopt_interface.cc"],
    hdrs = ["cos_theta_ipopt_interface.h"],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    deps = [
        "//cyber",
        "@adolc",
        "@ipopt",
    ],
)

cc_library(
    name = "fem_pos_deviation_smoother",
    srcs = ["fem_pos_deviation_smoother.cc"],
    hdrs = ["fem_pos_deviation_smoother.h"],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    deps = [
        ":fem_pos_deviation_ipopt_interface",
        ":fem_pos_deviation_osqp_interface",
        ":fem_pos_deviation_sqp_osqp_interface",
        "//cyber",
        "//modules/planning/proto/math:fem_pos_deviation_smoother_config_cc_proto",
        "@ipopt",
    ],
)

cc_library(
    name = "fem_pos_deviation_ipopt_interface",
    srcs = ["fem_pos_deviation_ipopt_interface.cc"],
    hdrs = ["fem_pos_deviation_ipopt_interface.h"],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    deps = [
        "//cyber",
        "@adolc",
        "@ipopt",
    ],
)

cc_library(
    name = "fem_pos_deviation_osqp_interface",
    srcs = ["fem_pos_deviation_osqp_interface.cc"],
    hdrs = ["fem_pos_deviation_osqp_interface.h"],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    deps = [
        "//cyber",
        "@osqp",
    ],
)

cc_library(
    name = "fem_pos_deviation_sqp_osqp_interface",
    srcs = ["fem_pos_deviation_sqp_osqp_interface.cc"],
    hdrs = ["fem_pos_deviation_sqp_osqp_interface.h"],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    deps = [
        "//cyber",
        "@osqp",
    ],
)

cpplint()
