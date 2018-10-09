load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "stop_sign_unprotected",
    srcs = [
        "stop_sign_unprotected.cc",
    ],
    hdrs = [
        "stop_sign_unprotected.h",
    ],
    deps = [
        "//external:gflags",
        "//cybertron",
        "//modules/common",
        "//modules/common/status",
        "//modules/common/util",
        "//modules/common/util:factory",
        "//modules/planning/common:speed_profile_generator",
        "//modules/planning/math/curve1d:quartic_polynomial_curve1d",
        "//modules/planning/proto:planning_proto",
        "//modules/planning/scenarios:scenario_lib",
        "//modules/planning/toolkits/optimizers:path_optimizer",
        "//modules/planning/toolkits/optimizers:speed_optimizer",
        "//modules/planning/toolkits/optimizers:task",
        "//modules/planning/toolkits/optimizers/dp_poly_path:dp_poly_path_optimizer",
        "//modules/planning/toolkits/optimizers/dp_st_speed:dp_st_speed_optimizer",
        "//modules/planning/toolkits/optimizers/path_decider",
        "//modules/planning/toolkits/optimizers/poly_st_speed:poly_st_speed_optimizer",
        "//modules/planning/toolkits/optimizers/qp_spline_path",
        "//modules/planning/toolkits/optimizers/qp_spline_st_speed:qp_spline_st_speed_optimizer",
        "//modules/planning/toolkits/optimizers/speed_decider",
        "@eigen",
    ],
)

cc_test(
    name = "stop_sign_unprotected_test",
    size = "small",
    srcs = [
        "stop_sign_unprotected_test.cc",
    ],
    data = [
        "//modules/planning:planning_conf",
    ],
    deps = [
        ":stop_sign_unprotected",
        "@gtest//:main",
    ],
)

cpplint()
