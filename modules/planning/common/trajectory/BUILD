package(default_visibility = ["//visibility:public"])

cc_library(
    name = "trajectory",
    hdrs = [
        "trajectory.h",
    ],
    deps = [
        "//external:gflags",
        "//modules/common/math",
        "//modules/common/proto:path_point_proto",
    ],
)

cc_library(
    name = "discretized_trajectory",
    srcs = [
        "discretized_trajectory.cc",
    ],
    hdrs = [
        "discretized_trajectory.h",
    ],
    deps = [
        ":trajectory",
        "//modules/common/proto:path_point_proto",
        "//modules/planning/common/path:path_point_util",
        "@eigen//:eigen",
    ],
)
