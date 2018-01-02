load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "constraint_checker",
    srcs = [
        "constraint_checker.cc",
    ],
    hdrs = [
        "constraint_checker.h",
    ],
    deps = [
        "//modules/common:log",
        "//modules/common/proto:pnc_point_proto",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common/trajectory",
        "//modules/planning/common/trajectory:discretized_trajectory",
    ],
)

cpplint()
