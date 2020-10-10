load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])
CONTROL_COPTS = ['-DMODULE_NAME=\\"control\\"']

cc_library(
    name = "controller_interface",
    srcs = [],
    hdrs = ["controller.h"],
    copts = CONTROL_COPTS,
    deps = [
        "//modules/control/common:dependency_injector",
    ],
)

cc_library(
    name = "lat_controller",
    srcs = ["lat_controller.cc"],
    hdrs = ["lat_controller.h"],
    copts = CONTROL_COPTS,
    deps = [
        ":controller_interface",
        "//cyber",
        "//cyber/time:clock",
        "//modules/common/configs:config_gflags",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/filters:digital_filter",
        "//modules/common/filters:digital_filter_coefficients",
        "//modules/common/filters:mean_filter",
        "//modules/common/math:euler_angles_zxy",
        "//modules/common/math:geometry",
        "//modules/common/math:lqr",
        "//modules/common/math:mpc_osqp",
        "//modules/common/status",
        "//modules/control/common:control_gflags",
        "//modules/control/common:interpolation_1d",
        "//modules/control/common:leadlag_controller",
        "//modules/control/common:mrac_controller",
        "//modules/control/common:trajectory_analyzer",
        "//modules/control/proto:calibration_table_cc_proto",
        "//modules/control/proto:control_cmd_cc_proto",
        "//modules/control/proto:control_conf_cc_proto",
        "//modules/control/proto:gain_scheduler_conf_cc_proto",
        "//modules/control/proto:input_debug_cc_proto",
        "//modules/control/proto:lat_controller_conf_cc_proto",
        "//modules/control/proto:lon_controller_conf_cc_proto",
        "//modules/control/proto:mpc_controller_conf_cc_proto",
        "//modules/control/proto:mrac_conf_cc_proto",
        "//modules/control/proto:pad_msg_cc_proto",
        "@com_google_absl//absl/strings",
        "@eigen",
    ],
)

cc_library(
    name = "lon_controller",
    srcs = ["lon_controller.cc"],
    hdrs = ["lon_controller.h"],
    copts = CONTROL_COPTS,
    deps = [
        ":controller_interface",
        "//cyber",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/filters:digital_filter",
        "//modules/common/filters:digital_filter_coefficients",
        "//modules/common/status",
        "//modules/control/common:control_gflags",
        "//modules/control/common:interpolation_2d",
        "//modules/control/common:leadlag_controller",
        "//modules/control/common:pid_controller",
        "//modules/control/common:trajectory_analyzer",
        "//modules/control/proto:calibration_table_cc_proto",
        "//modules/control/proto:control_cmd_cc_proto",
        "//modules/control/proto:control_conf_cc_proto",
        "//modules/control/proto:gain_scheduler_conf_cc_proto",
        "//modules/control/proto:input_debug_cc_proto",
        "//modules/control/proto:lat_controller_conf_cc_proto",
        "//modules/control/proto:lon_controller_conf_cc_proto",
        "//modules/control/proto:mpc_controller_conf_cc_proto",
        "//modules/control/proto:mrac_conf_cc_proto",
        "//modules/control/proto:pad_msg_cc_proto",
        "//modules/localization/common:localization_common",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "mpc_controller",
    srcs = ["mpc_controller.cc"],
    hdrs = ["mpc_controller.h"],
    copts = CONTROL_COPTS,
    deps = [
        ":controller_interface",
        "//cyber",
        "//cyber/time:clock",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/filters:digital_filter",
        "//modules/common/filters:digital_filter_coefficients",
        "//modules/common/filters:mean_filter",
        "//modules/common/math:euler_angles_zxy",
        "//modules/common/math:geometry",
        "//modules/common/math:lqr",
        "//modules/common/math:mpc_osqp",
        "//modules/common/proto:geometry_cc_proto",
        "//modules/common/status",
        "//modules/control/common:control_gflags",
        "//modules/control/common:interpolation_1d",
        "//modules/control/common:interpolation_2d",
        "//modules/control/common:trajectory_analyzer",
        "//modules/control/proto:calibration_table_cc_proto",
        "//modules/control/proto:control_cmd_cc_proto",
        "//modules/control/proto:control_conf_cc_proto",
        "//modules/control/proto:gain_scheduler_conf_cc_proto",
        "//modules/control/proto:input_debug_cc_proto",
        "//modules/control/proto:lat_controller_conf_cc_proto",
        "//modules/control/proto:lon_controller_conf_cc_proto",
        "//modules/control/proto:mpc_controller_conf_cc_proto",
        "//modules/control/proto:mrac_conf_cc_proto",
        "//modules/control/proto:pad_msg_cc_proto",
        "@com_google_absl//absl/strings",
        "@eigen",
    ],
)

cc_library(
    name = "controller",
    copts = CONTROL_COPTS,
    deps = [
        ":controller_agent",
        ":controller_interface",
        ":lat_controller",
        ":lon_controller",
        ":mpc_controller",
        "@eigen",
    ],
)

cc_library(
    name = "controller_agent",
    srcs = ["controller_agent.cc"],
    hdrs = ["controller_agent.h"],
    copts = CONTROL_COPTS,
    deps = [
        ":controller_interface",
        ":lat_controller",
        ":lon_controller",
        ":mpc_controller",
        "//cyber",
        "//cyber/time:clock",
        "//modules/common/util:factory",
        "//modules/control/common:dependency_injector",
        "//modules/control/proto:control_cmd_cc_proto",
        "//modules/planning/proto:planning_cc_proto",
        "@osqp",
    ],
)

cc_test(
    name = "lon_controller_test",
    size = "small",
    srcs = ["lon_controller_test.cc"],
    copts = ["-fno-access-control"],
    data = ["//modules/control/testdata:control_testdata"],
    deps = [
        ":lon_controller",
        "//cyber",
        "//cyber/time:clock",
        "//modules/common/util",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/control/common:control_gflags",
        "//modules/control/proto:control_conf_cc_proto",
        "//modules/planning/proto:planning_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "lat_controller_test",
    size = "small",
    srcs = ["lat_controller_test.cc"],
    copts = ["-fno-access-control"],
    data = ["//modules/control/testdata:control_testdata"],
    deps = [
        ":lat_controller",
        "//cyber",
        "//cyber/time:clock",
        "//modules/common/util",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/control/common:control_gflags",
        "//modules/planning/proto:planning_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "mpc_controller_test",
    size = "small",
    srcs = ["mpc_controller_test.cc"],
    copts = ["-fno-access-control"],
    data = ["//modules/control/testdata:control_testdata"],
    deps = [
        ":mpc_controller",
        "//cyber",
        "//cyber/time:clock",
        "//modules/common/util",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/control/common:control_gflags",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
