load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package", "apollo_cc_test", "apollo_plugin")

package(default_visibility = ["//visibility:public"])

CONTROL_COPTS = ['-DMODULE_NAME=\\"control\\"']

apollo_plugin(
    name = "liblon_controller.so",
    srcs = ["lon_controller.cc"],
    hdrs = ["lon_controller.h"],
    description = ":plugins.xml",
    deps = [
        "//cyber",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/filters",
        "//modules/common/status",
        "//modules/control/control_component/common:control_gflags",
        "//modules/control/control_component/controller_task_base:control_task",
        "//modules/control/control_component/controller_task_base/common:interpolation_2d",
        "//modules/control/control_component/controller_task_base/common:leadlag_controller",
        "//modules/control/control_component/controller_task_base/common:pid_controller",
        "//modules/control/control_component/controller_task_base/common:trajectory_analyzer",
        "//modules/control/control_component/controller_task_base/proto:calibration_table_cc_proto",
        "//modules/common_msgs/control_msgs:control_cmd_cc_proto",
        "//modules/control/control_component/controller_task_base/proto:gain_scheduler_conf_cc_proto",
        "//modules/common_msgs/control_msgs:input_debug_cc_proto",
        "//modules/control/controllers/lon_based_pid_controller/proto:lon_based_pid_controller_conf_cc_proto",
        "//modules/control/control_component/controller_task_base/proto:mrac_conf_cc_proto",
        "//modules/common_msgs/control_msgs:control_pad_msg_cc_proto",
        "//modules/localization/common:localization_gflags",
        "@com_google_absl//:absl",
    ],
)

apollo_cc_test(
    name = "lon_controller_test",
    size = "small",
    srcs = ["lon_controller_test.cc"],
    copts = ["-fno-access-control"],
    data = ["longitudinal_controller_test"],
    deps = [
        ":lon_controller_lib",
        "//cyber",
        "//modules/common/util:common_util",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/control/control_component/common:control_gflags",
        "//modules/control/control_component/controller_task_base:control_task",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

filegroup(
    name = "longitudinal_controller_test",
    srcs = glob([
        "longitudinal_controller_test/**",
    ]) + glob(["conf/*"]),
)

apollo_package()
cpplint()
