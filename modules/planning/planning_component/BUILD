load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/proto:proto.bzl", "apollo_py_binary")

package(default_visibility = ["//visibility:public"])

apollo_component(
    name = "libplanning_component.so",
    srcs = [
        "planning_component.cc",
        "navi_planning.cc",
        "on_lane_planning.cc",
        "planning_base.cc",
    ],
    hdrs = [
        "planning_component.h",
        "navi_planning.h",
        "on_lane_planning.h",
        "planning_base.h",
        "integration_tests/planning_test_base.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    deps = [
        "//modules/planning/planning_base:apollo_planning_planning_base",
        "//modules/planning/planning_interface_base:apollo_planning_planning_interface_base",
        "//modules/common_msgs/external_command_msgs:lane_follow_command_cc_proto",
        "//modules/common_msgs/external_command_msgs:command_status_cc_proto",
        "//modules/common_msgs/control_msgs:control_interactive_msg_cc_proto",
        "//cyber",
    ],
)

apollo_cc_test(
    name = "planning_test_base",
    srcs = ["integration_tests/planning_test_base.cc"],
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/configs:config_gflags",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "DO_NOT_IMPORT_planning_component",
        "@com_google_googletest//:gtest",
    ],
    linkstatic = True,
)

filegroup(
    name = "planning_conf",
    srcs = glob([
        "conf/**",
    ]),
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "dag/*.dag",
        "launch/*.launch",
    ]) + [":planning_conf"],
)

apollo_package()

cpplint()
