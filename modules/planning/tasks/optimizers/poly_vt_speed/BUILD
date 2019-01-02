load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "piecewise_poly_speed_curve",
    srcs = [
        "piecewise_poly_speed_curve.cc",
    ],
    hdrs = [
        "piecewise_poly_speed_curve.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/common/proto:pnc_point_proto",
        "//modules/planning/math/curve1d:cubic_polynomial_curve1d",
        "//modules/planning/math/curve1d:quartic_polynomial_curve1d",
        "//modules/planning/math/curve1d:quintic_polynomial_curve1d",
    ],
)

cc_test(
    name = "piecewise_poly_speed_curve_test",
    size = "small",
    srcs = [
        "piecewise_poly_speed_curve_test.cc",
    ],
    deps = [
        ":piecewise_poly_speed_curve",
        "@gtest//:main",
    ],
)

cc_library(
    name = "piecewise_poly_speed_profile",
    srcs = [
        "piecewise_poly_speed_profile.cc",
    ],
    hdrs = [
        "piecewise_poly_speed_profile.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        ":piecewise_poly_speed_curve",
        "//modules/common/proto:pnc_point_proto",
    ],
)

cc_test(
    name = "piecewise_poly_speed_profile_test",
    size = "small",
    srcs = [
        "piecewise_poly_speed_profile_test.cc",
    ],
    deps = [
        ":piecewise_poly_speed_profile",
        "@gtest//:main",
    ],
)

cc_library(
    name = "piecewise_poly_vt_speed_sampler",
    srcs = [
        "piecewise_poly_vt_speed_sampler.cc",
    ],
    hdrs = [
        "piecewise_poly_vt_speed_sampler.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        ":piecewise_poly_speed_curve",
        ":piecewise_poly_speed_profile",
        "//cyber/common:log",
        "//modules/common/proto:pnc_point_proto",
        "//modules/planning/proto:poly_vt_speed_config_proto",
    ],
)

cc_test(
    name = "piecewise_poly_vt_speed_sampler_test",
    size = "small",
    srcs = [
        "piecewise_poly_vt_speed_sampler_test.cc",
    ],
    deps = [
        ":piecewise_poly_vt_speed_sampler",
        "@gtest//:main",
    ],
)

cc_library(
    name = "poly_vt_speed_optimizer",
    srcs = [
        "poly_vt_speed_optimizer.cc",
    ],
    hdrs = [
        "poly_vt_speed_optimizer.h",
    ],
    deps = [
        ":piecewise_poly_vt_speed_sampler",
        "//cyber/common:log",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/configs/proto:vehicle_config_proto",
        "//modules/common/proto:pnc_point_proto",
        "//modules/common/status",
        "//modules/localization/proto:pose_proto",
        "//modules/planning/common:path_decision",
        "//modules/planning/common:reference_line_info",
        "//modules/planning/common:speed_limit",
        "//modules/planning/common/path:path_data",
        "//modules/planning/common/speed:speed_data",
        "//modules/planning/proto:planning_config_proto",
        "//modules/planning/proto:planning_proto",
        "//modules/planning/proto:poly_vt_speed_config_proto",
        "//modules/planning/tasks:task",
        "//modules/planning/tasks/optimizers/st_graph:st_boundary_mapper",
        "//modules/planning/tasks/optimizers/st_graph:st_graph_data",
        "//modules/planning/tuning:autotuning_raw_feature_generator",
        "//modules/planning/tuning/speed_model:autotuning_speed_feature_builder",
        "//modules/planning/tuning/speed_model:autotuning_speed_mlp_model",
    ],
)

cpplint()
