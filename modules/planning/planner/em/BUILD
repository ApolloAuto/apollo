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
        "//modules/common/proto:path_point_proto",
        "//modules/common/util",
        "//modules/common/util:factory",
        "//modules/common/vehicle_state",
        "//modules/planning/common:planning_common",
        "//modules/planning/math/curve1d:quartic_polynomial_curve1d",
        "//modules/planning/planner",
        "@eigen//:eigen",
    ],
)

cpplint()
