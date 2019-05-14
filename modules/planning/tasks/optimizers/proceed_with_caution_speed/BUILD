load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "proceed_with_caution_speed_generator",
    srcs = [
        "proceed_with_caution_speed_generator.cc",
    ],
    hdrs = [
        "proceed_with_caution_speed_generator.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//cyber/common:log",
        "//modules/planning/common:planning_context",
        "//modules/planning/common:speed_profile_generator",
        "//modules/planning/proto:planning_config_proto",
        "//modules/planning/tasks/deciders/speed_bounds_decider:st_boundary_mapper",
        "//modules/planning/tasks/optimizers:speed_optimizer",
    ],
)

cc_test(
    name = "proceed_with_caution_speed_generator_test",
    size = "small",
    srcs = [
        "proceed_with_caution_speed_generator_test.cc",
    ],
    deps = [
        ":proceed_with_caution_speed_generator",
        "@gtest//:main",
    ],
)

cpplint()
