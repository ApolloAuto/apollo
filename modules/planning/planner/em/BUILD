load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "decider",
    srcs = [
        "decider.cc",
    ],
    hdrs = [
        "decider.h",
    ],
    deps = [
        "//external:gflags",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common:log",
        "//modules/common/status",
        "//modules/common/vehicle_state",
        "//modules/planning/common:planning_common",
        "//modules/planning/proto:planning_proto",
        "//modules/planning/reference_line",
        "@eigen//:eigen",
    ],
)

cc_library(
    name = "em_planner",
    srcs = [
        "em_planner.cc",
    ],
    hdrs = [
        "em_planner.h",
    ],
    deps = [
        ":decider",
        "//external:gflags",
        "//modules/common:log",
        "//modules/common/proto:pnc_point_proto",
        "//modules/common/status",
        "//modules/common/time",
        "//modules/common/util",
        "//modules/common/util:factory",
        "//modules/common/vehicle_state",
        "//modules/map/hdmap",
        "//modules/planning/common:planning_common",
        "//modules/planning/math/curve1d:quartic_polynomial_curve1d",
        "//modules/planning/optimizer",
        "//modules/planning/optimizer:path_optimizer",
        "//modules/planning/optimizer:speed_optimizer",
        "//modules/planning/optimizer/dp_poly_path:dp_poly_path_optimizer",
        "//modules/planning/optimizer/dp_st_speed:dp_st_speed_optimizer",
        "//modules/planning/optimizer/qp_spline_path",
        "//modules/planning/optimizer/qp_spline_st_speed:qp_spline_st_speed_optimizer",
        "//modules/planning/planner",
        "//modules/planning/proto:planning_proto",
        "//modules/planning/reference_line",
        "//modules/planning/reference_line:reference_line_smoother",
        "@eigen//:eigen",
    ],
)

cpplint()
