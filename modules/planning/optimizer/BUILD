load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "optimizer",
    srcs = [
        "optimizer.cc",
    ],
    hdrs = [
        "optimizer.h",
    ],
)

cc_library(
    name = "path_optimizer",
    srcs = [
        "path_optimizer.cc",
    ],
    hdrs = [
        "path_optimizer.h",
    ],
    deps = [
        ":optimizer",
        "//modules/common/status",
        "//modules/planning/common:data_center",
        "//modules/planning/common:decision_data",
        "//modules/planning/common:planning_data",
        "//modules/planning/common/path:path_data",
        "//modules/planning/common/speed:speed_data",
        "@eigen//:eigen",
    ],
)

cc_library(
    name = "speed_optimizer",
    srcs = [
        "speed_optimizer.cc",
    ],
    hdrs = [
        "speed_optimizer.h",
    ],
    deps = [
        ":optimizer",
        "//modules/common/status",
        "//modules/planning/common:data_center",
        "//modules/planning/common:decision_data",
        "//modules/planning/common:planning_data",
        "//modules/planning/common/path:path_data",
        "//modules/planning/common/speed:speed_data",
        "@eigen//:eigen",
    ],
)

cpplint()
