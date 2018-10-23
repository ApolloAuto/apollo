load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "side_pass",
    srcs = [
        "side_pass_scenario.cc",
        "side_pass_stage.cc",
    ],
    hdrs = [
        "side_pass_scenario.h",
        "side_pass_stage.h",
    ],
    deps = [
        "//cyber/common:log",
        "//external:gflags",
        "//modules/common",
        "//modules/common/proto:pnc_point_proto",
        "//modules/common/status",
        "//modules/common/time",
        "//modules/common/util",
        "//modules/common/util:factory",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/map/hdmap",
        "//modules/planning/common:planning_common",
        "//modules/planning/common:speed_profile_generator",
        "//modules/planning/proto:planning_proto",
        "//modules/planning/reference_line",
        "//modules/planning/reference_line:qp_spline_reference_line_smoother",
        "//modules/planning/scenarios:scenario",
        "//modules/planning/toolkits/optimizers/proceed_with_caution_speed:proceed_with_caution_speed_generator",
        "@eigen",
    ],
)

cc_test(
    name = "side_pass_scenario_test",
    size = "small",
    srcs = [
        "side_pass_scenario_test.cc",
    ],
    data = [
        "//modules/planning:planning_conf",
    ],
    deps = [
        ":side_pass",
        "@gtest//:main",
    ],
)

cpplint()
