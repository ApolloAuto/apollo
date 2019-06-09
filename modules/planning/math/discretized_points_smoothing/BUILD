load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "cos_theta_smoother",
    srcs = [
        "cos_theta_smoother.cc",
    ],
    hdrs = [
        "cos_theta_smoother.h",
    ],
    deps = [
        ":cos_theta_ipopt_interface",
        "//cyber/common:log",
        "//modules/planning/proto:cos_theta_smoother_config_proto",
        "@ipopt",
    ],
)

cc_library(
    name = "cos_theta_ipopt_interface",
    srcs = [
        "cos_theta_ipopt_interface.cc",
    ],
    hdrs = [
        "cos_theta_ipopt_interface.h",
    ],
    deps = [
        "//cyber/common:log",
        "@adolc",
        "@ipopt",
    ],
)

cc_library(
    name = "fem_pos_deviation_smoother",
    srcs = [
        "fem_pos_deviation_smoother.cc",
    ],
    hdrs = [
        "fem_pos_deviation_smoother.h",
    ],
    deps = [
        "//cyber/common:log",
        "@osqp",
    ],
)

cpplint()
