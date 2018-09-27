load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "node3d",
    srcs = ["node3d.cc"],
    hdrs = ["node3d.h"],
    deps = ["//framework:cybertron",
            "//modules/common/math",
            "//modules/planning/common:obstacle",
            "//modules/planning/constraint_checker:collision_checker",
            "//modules/planning/proto:planner_open_space_config_proto",],
)

cc_library(
    name = "reeds_shepp_path",
    srcs = ["reeds_shepp_path.cc"],
    hdrs = ["reeds_shepp_path.h"],
    deps = [
        "//framework:cybertron",
        "//modules/common/math",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/planning/proto:planner_open_space_config_proto",
        "//modules/planning/open_space:node3d"],
)

cc_library(
    name = "open_space_utils",
    deps = ["//modules/planning/open_space:reeds_shepp_path",
            "//modules/planning/open_space:node3d",]
)

cc_test(
    name = "reeds_shepp_path_test",
    size = "small",
    srcs = ["reeds_shepp_path_test.cc"],
    deps = [
        "//framework:cybertron",
        "//modules/planning/open_space:open_space_utils",
        "//modules/common/math",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/planning/proto:planner_open_space_config_proto",
        "@gtest//:main",],
)

cc_test(
    name = "node3d_test",
    size = "small",
    srcs = ["node3d_test.cc"],
    deps = [
        "//framework:cybertron",
        "//modules/planning/open_space:open_space_utils",
        "@gtest//:main",
    ],
)
cpplint()