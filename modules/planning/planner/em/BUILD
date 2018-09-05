load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "em_planner",
    srcs = [
        "em_planner.cc",
    ],
    hdrs = [
        "em_planner.h",
    ],
    deps = [
        "//external:gflags",
        "//modules/common:log",
        "//modules/common/proto:pnc_point_proto",
        "//modules/common/status",
        "//modules/common/time",
        "//modules/common/util",
        "//modules/common/util:factory",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/map/hdmap",
        "//modules/planning/common:planning_common",
        "//modules/planning/constraint_checker",
        "//modules/planning/math/curve1d:quartic_polynomial_curve1d",
        "//modules/planning/planner",
        "//modules/planning/proto:planning_proto",
        "//modules/planning/reference_line",
        "//modules/planning/reference_line:qp_spline_reference_line_smoother",
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
        "@eigen//:eigen",
    ],
)

cc_test(
    name = "em_planner_test",
    size = "small",
    srcs = [
        "em_planner_test.cc",
    ],
    deps = [
        ":em_planner",
        "@gtest//:main",
    ],
)

cpplint()
