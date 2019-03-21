load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "util_lib",
    srcs = [
        "util.cc",
    ],
    hdrs = [
        "util.h",
    ],
    deps = [
        "//cyber/common:log",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/common/vehicle_state/proto:vehicle_state_proto",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:reference_line_info",
        "//modules/routing:routing_lib",
        "//modules/common/util",
        "//modules/planning/proto:planning_proto",   
    ],
)

cpplint()
