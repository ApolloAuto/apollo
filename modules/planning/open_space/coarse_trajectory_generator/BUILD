load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "node3d",
    srcs = ["node3d.cc"],
    hdrs = ["node3d.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//cyber/common:log",
        "//modules/common/math",
        "//modules/planning/common:obstacle",
        "//modules/planning/constraint_checker:collision_checker",
        "//modules/planning/proto:planner_open_space_config_proto",
    ],
)

cc_library(
    name = "reeds_shepp_path",
    srcs = ["reeds_shepp_path.cc"],
    hdrs = ["reeds_shepp_path.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\"", "-fopenmp"],
    deps = [
        "//modules/planning/common:planning_gflags",
        "//cyber/common:log",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/math",
        "//modules/planning/open_space/coarse_trajectory_generator:node3d",
        "//modules/planning/proto:planner_open_space_config_proto",
        "@adolc",
    ],
)

cc_library(
    name = "grid_search",
    srcs = [
        "grid_search.cc",
    ],
    hdrs = [
        "grid_search.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//cyber/common:log",
        "//modules/common/math",
        "//modules/planning/proto:planner_open_space_config_proto",
    ],
)

cc_library(
    name = "open_space_utils",
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/planning/open_space/coarse_trajectory_generator:node3d",
        "//modules/planning/open_space/coarse_trajectory_generator:reeds_shepp_path",
        "//modules/planning/open_space/coarse_trajectory_generator:grid_search",

    ],
)

cc_library(
    name = "hybrid_a_star",
    srcs = [
        "hybrid_a_star.cc",
    ],
    hdrs = [
        "hybrid_a_star.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "open_space_utils",
        "//cyber/common:log",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/planning/common:obstacle",
        "//modules/planning/proto:planner_open_space_config_proto",
    ],
)

cc_test(
    name = "reeds_shepp_path_test",
    size = "small",
    srcs = ["reeds_shepp_path_test.cc"],
    deps = [
        "open_space_utils",
        "//cyber/common:log",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/math",
        "//modules/planning/proto:planner_open_space_config_proto",
        "@gtest//:main",
    ],
)

cc_test(
    name = "node3d_test",
    size = "small",
    srcs = ["node3d_test.cc"],
    deps = [
        "open_space_utils",
        "//cyber/common:log",
        "@gtest//:main",
    ],
)

cc_test(
    name = "hybrid_a_star_test",
    size = "small",
    srcs = [
        "hybrid_a_star_test.cc",
    ],
    deps = [
        "hybrid_a_star",
        "@gtest//:main",
    ],
)

cpplint()
