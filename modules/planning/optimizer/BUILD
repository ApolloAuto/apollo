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
    deps = [
        "//modules/common/status",
        "//modules/planning/common:frame",
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

cc_library(
    name = "optimizer_test_base",
    srcs = [
        "optimizer_test_base.cc",
    ],
    hdrs = [
        "optimizer_test_base.h",
    ],
    data = [
        "//modules/map:map_data",
        "//modules/planning:planning_testdata",
    ],
    deps = [
        "//modules/common:log",
        "//modules/common/adapters:adapter_manager",
        "//modules/common/util",
        "//modules/common/vehicle_state",
        "//modules/planning:planning_impl",
        "//modules/planning/common:data_center",
        "//modules/planning/proto:dp_poly_path_config_proto",
        "//modules/planning/proto:dp_st_speed_config_proto",
        "@gtest//:gtest",
    ],
)

cpplint()
