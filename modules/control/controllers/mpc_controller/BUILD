load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package", "apollo_cc_test", "apollo_plugin")

package(default_visibility = ["//visibility:public"])

CONTROL_COPTS = ['-DMODULE_NAME=\\"control\\"']

apollo_plugin(
    name = "libmpc_controller.so",
    srcs = ["mpc_controller.cc"],
    hdrs = ["mpc_controller.h"],
    description = ":plugins.xml",
    copts = CONTROL_COPTS,
    deps = [
        "//modules/control/control_component/controller_task_base:control_task",
        "//cyber",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/filters",
        "//modules/common/math",
        "//modules/common_msgs/basic_msgs:geometry_cc_proto",
        "//modules/common/status",
        "//modules/control/control_component/common:control_gflags",
        "//modules/control/control_component/controller_task_base/common:interpolation_1d",
        "//modules/control/control_component/controller_task_base/common:interpolation_2d",
        "//modules/control/control_component/controller_task_base/common:trajectory_analyzer",
        "//modules/control/control_component/proto:calibration_table_cc_proto",
        "//modules/common_msgs/control_msgs:control_cmd_cc_proto",
        "//modules/control/control_component/proto:gain_scheduler_conf_cc_proto",
        "//modules/common_msgs/control_msgs:input_debug_cc_proto",
        "//modules/control/controllers/mpc_controller/proto:mpc_controller_cc_proto",
        "//modules/control/control_component/proto:mrac_conf_cc_proto",
        "//modules/common_msgs/control_msgs:control_pad_msg_cc_proto",
        "@com_google_absl//:absl",
        "@eigen",
    ],
)

apollo_cc_test(
    name = "mpc_controller_test",
    size = "small",
    srcs = ["mpc_controller_test.cc"],
    copts = ["-fno-access-control"],
    data = [
        "mpc_controller_test_data",
    ],
    deps = [
        ":mpc_controller_lib",
        "//modules/control/control_component/controller_task_base:control_task",
        "@com_google_googletest//:gtest_main",
    ],
)

filegroup(
    name = "mpc_controller_test_data",
    srcs = glob([
        "mpc_controller_test_data/**",
    ]) + glob(["conf/*"]),
)

apollo_package()

cpplint()
