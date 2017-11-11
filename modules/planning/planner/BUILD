load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "planner",
    hdrs = [
        "planner.h",
    ],
    deps = [
        "//modules/common/proto:pnc_point_proto",
        "//modules/common/status",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/planning/common:frame",
        "//modules/planning/common:reference_line_info",
        "//modules/planning/proto:planning_proto",
    ],
)

cpplint()
