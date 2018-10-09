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
    deps = [
        "//framework:cybertron",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/configs/proto:vehicle_config_proto",
        "//modules/common/proto:pnc_point_proto",
        "//modules/localization/proto:pose_proto",
        "//modules/map/hdmap",
        "//modules/planning/proto:planning_config_proto",
        "//modules/planning/toolkits/optimizers:speed_optimizer",
        "//modules/planning/toolkits/optimizers/st_graph:st_boundary_mapper",
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
