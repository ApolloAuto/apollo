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

cc_library(
    name = "publishable_trajectory",
    srcs = [
        "publishable_trajectory.cc",
    ],
    hdrs = [
        "publishable_trajectory.h",
    ],
    deps = [
        ":discretized_trajectory",
        "//modules/planning/proto:planning_proto",
    ],
)

cc_library(
    name = "prediction_trajectory",
    srcs = [
        "prediction_trajectory.cc",
    ],
    hdrs = [
        "prediction_trajectory.h",
    ],
    deps = [
        ":discretized_trajectory",
        "//modules/common/proto:path_point_proto",
    ],
)
