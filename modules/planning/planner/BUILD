load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "planner",
    hdrs = [
        "planner.h",
    ],
    deps = [
        "//modules/common/proto:path_point_proto",
        "//modules/common/status",
        "//modules/common/vehicle_state",
        "//modules/planning/proto:planning_proto",
    ],
)

cpplint()
