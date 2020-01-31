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
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
        "-fopenmp",
    ],
    deps = [
        "//cyber/common:log",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/math",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/open_space/coarse_trajectory_generator:node3d",
        "//modules/planning/proto:planner_open_space_config_proto",
        "//third_party:adolc",
    ],
)

cc_library(
    name = "grid_search",
    srcs = ["grid_search.cc"],
    hdrs = ["grid_search.h"],
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
        "//modules/planning/open_space/coarse_trajectory_generator:grid_search",
        "//modules/planning/open_space/coarse_trajectory_generator:node3d",
        "//modules/planning/open_space/coarse_trajectory_generator:reeds_shepp_path",
    ],
)

cc_library(
    name = "hybrid_a_star",
    srcs = ["hybrid_a_star.cc"],
    hdrs = ["hybrid_a_star.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        ":open_space_utils",
        "//cyber/common:log",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/planning/common:obstacle",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/math/piecewise_jerk:piecewise_jerk_speed_problem",
        "//modules/planning/proto:planner_open_space_config_proto",
    ],
)

cc_test(
    name = "reeds_shepp_path_test",
    size = "small",
    srcs = ["reeds_shepp_path_test.cc"],
    deps = [
        ":open_space_utils",
        "//cyber/common:log",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/math",
        "//modules/planning/proto:planner_open_space_config_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "node3d_test",
    size = "small",
    srcs = ["node3d_test.cc"],
    deps = [
        ":open_space_utils",
        "//cyber/common:log",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "hybrid_a_star_test",
    size = "small",
    srcs = ["hybrid_a_star_test.cc"],
    deps = [
        ":hybrid_a_star",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
