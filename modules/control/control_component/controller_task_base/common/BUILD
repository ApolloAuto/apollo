load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_package", "apollo_cc_test")

package(default_visibility = ["//visibility:public"])

CONTROL_COPTS = ['-DMODULE_NAME=\\"control\\"']

apollo_cc_library(
    name = "controllers_control_task_base_common",
    hdrs = glob(["*.h"]),
    visibility = ["//visibility:public"],
    deps = [
        ":dependency_injector",
        ":hysteresis_filter",
        ":interpolation_1d",
        ":interpolation_2d",
        ":leadlag_controller",
        ":mrac_controller",
        ":pid_BC_controller",
        ":pid_IC_controller",
        ":pid_controller",
        ":trajectory_analyzer",
        "//cyber",
        "//modules/common/math",
        "//modules/common/status",
        "//modules/common_msgs/config_msgs:vehicle_config_cc_proto",
        "//modules/control/control_component/proto:leadlag_conf_cc_proto",
        "//modules/control/control_component/proto:mrac_conf_cc_proto",
        "//modules/control/control_component/proto:pid_conf_cc_proto",
        "@com_google_absl//:absl",
        "@eigen",
    ],
)

apollo_cc_library(
    name = "dependency_injector",
    hdrs = ["dependency_injector.h"],
    deps = [
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/common_msgs/control_msgs:control_interactive_msg_proto",
        "//modules/common_msgs/external_command_msgs:command_status_proto",
        "//modules/control/control_component/proto:control_debug_proto",
    ],
)

apollo_cc_library(
    name = "hysteresis_filter",
    srcs = ["hysteresis_filter.cc"],
    hdrs = ["hysteresis_filter.h"],
    copts = CONTROL_COPTS,
)

apollo_cc_library(
    name = "interpolation_1d",
    srcs = ["interpolation_1d.cc"],
    hdrs = ["interpolation_1d.h"],
    copts = CONTROL_COPTS,
    deps = [
        "//cyber",
        "@eigen",
    ],
)

apollo_cc_library(
    name = "interpolation_2d",
    srcs = ["interpolation_2d.cc"],
    hdrs = ["interpolation_2d.h"],
    copts = CONTROL_COPTS,
    deps = [
        "//cyber",
        "//modules/control/control_component/common:control_gflags",
    ],
)

apollo_cc_library(
    name = "leadlag_controller",
    srcs = ["leadlag_controller.cc"],
    hdrs = ["leadlag_controller.h"],
    copts = CONTROL_COPTS,
    deps = [
        "//cyber",
        "//modules/control/control_component/proto:leadlag_conf_cc_proto",
    ],
)

apollo_cc_library(
    name = "mrac_controller",
    srcs = ["mrac_controller.cc"],
    hdrs = ["mrac_controller.h"],
    copts = CONTROL_COPTS,
    deps = [
        "//cyber",
        "//modules/common/math",
        "//modules/common/status",
        "//modules/common_msgs/config_msgs:vehicle_config_cc_proto",
        "//modules/control/control_component/proto:mrac_conf_cc_proto",
        "@com_google_absl//:absl",
        "@eigen",
    ],
)

apollo_cc_library(
    name = "pid_controller",
    srcs = ["pid_controller.cc"],
    hdrs = ["pid_controller.h"],
    copts = CONTROL_COPTS,
    deps = [
        "//cyber",
        "//modules/control/control_component/proto:pid_conf_cc_proto",
    ],
)

apollo_cc_library(
    name = "pid_BC_controller",
    srcs = ["pid_BC_controller.cc"],
    hdrs = ["pid_BC_controller.h"],
    copts = CONTROL_COPTS,
    deps = [
        ":pid_controller",
        "//cyber",
        "//modules/common/math",
        "//modules/control/control_component/proto:pid_conf_cc_proto",
    ],
)

apollo_cc_library(
    name = "pid_IC_controller",
    srcs = ["pid_IC_controller.cc"],
    hdrs = ["pid_IC_controller.h"],
    copts = CONTROL_COPTS,
    deps = [
        ":pid_controller",
        "//cyber",
        "//modules/common/math",
        "//modules/control/control_component/proto:pid_conf_cc_proto",
    ],
)

apollo_cc_library(
    name = "trajectory_analyzer",
    srcs = ["trajectory_analyzer.cc"],
    hdrs = ["trajectory_analyzer.h"],
    copts = CONTROL_COPTS,
    deps = [
        "//cyber",
        "//modules/common/math",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/common_msgs/basic_msgs:pnc_point_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/control/control_component/common:control_gflags",
        "@eigen",
    ],
)

apollo_cc_test(
    name = "hysteresis_filter_test",
    size = "small",
    srcs = ["hysteresis_filter_test.cc"],
    deps = [
        ":hysteresis_filter",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "interpolation_1d_test",
    size = "small",
    srcs = ["interpolation_1d_test.cc"],
    data = ["//modules/control/control_component:test_data"],
    deps = [
        ":interpolation_1d",
        "//cyber",
        "//modules/control/controllers/lat_based_lqr_controller/proto:lat_based_lqr_controller_conf_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "interpolation_2d_test",
    size = "small",
    srcs = ["interpolation_2d_test.cc"],
    data = ["//modules/control/control_component:test_data"],
    deps = [
        ":interpolation_2d",
        "//cyber",
        "//modules/control/control_component/proto:calibration_table_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "leadlag_controller_test",
    size = "small",
    srcs = ["leadlag_controller_test.cc"],
    data = ["//modules/control/control_component:test_data"],
    deps = [
        ":leadlag_controller",
        "//cyber",
        "//modules/control/control_component/proto:leadlag_conf_cc_proto",
        "//modules/control/controllers/lon_based_pid_controller/proto:lon_based_pid_controller_conf_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "mrac_controller_test",
    size = "small",
    srcs = ["mrac_controller_test.cc"],
    data = ["//modules/control/control_component:test_data"],
    deps = [
        ":mrac_controller",
        "//cyber",
        "//modules/common_msgs/config_msgs:vehicle_config_cc_proto",
        "//modules/control/control_component/proto:mrac_conf_cc_proto",
        "//modules/control/controllers/lat_based_lqr_controller/proto:lat_based_lqr_controller_conf_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "pid_controller_test",
    size = "small",
    srcs = ["pid_controller_test.cc"],
    data = ["//modules/control/control_component:test_data"],
    deps = [
        ":pid_controller",
        "//cyber",
        "//modules/control/control_component/proto:pid_conf_cc_proto",
        "//modules/control/controllers/lon_based_pid_controller/proto:lon_based_pid_controller_conf_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "pid_BC_controller_test",
    size = "small",
    srcs = ["pid_BC_controller_test.cc"],
    data = ["//modules/control/control_component:test_data"],
    deps = [
        ":pid_BC_controller",
        "//cyber",
        "//modules/control/control_component/proto:pid_conf_cc_proto",
        "//modules/control/controllers/lon_based_pid_controller/proto:lon_based_pid_controller_conf_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "pid_IC_controller_test",
    size = "small",
    srcs = ["pid_IC_controller_test.cc"],
    data = ["//modules/control/control_component:test_data"],
    deps = [
        ":pid_IC_controller",
        "//cyber",
        "//modules/control/control_component/proto:pid_conf_cc_proto",
        "//modules/control/controllers/lon_based_pid_controller/proto:lon_based_pid_controller_conf_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "trajectory_analyzer_test",
    size = "small",
    srcs = ["trajectory_analyzer_test.cc"],
    deps = [
        ":trajectory_analyzer",
        "//cyber",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_package()

cpplint()
