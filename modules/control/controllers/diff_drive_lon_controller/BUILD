load("//tools:apollo_package.bzl", "apollo_cc_test", "apollo_package", "apollo_plugin")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

CONTROL_COPTS = ['-DMODULE_NAME=\\"control\\"']

apollo_plugin(
    name = "libdiff_drive_lon_controller.so",
    srcs = ["diff_drive_lon_controller.cc"],
    hdrs = ["diff_drive_lon_controller.h"],
    copts = CONTROL_COPTS,
    description = ":plugins.xml",
    deps = [
        "//cyber",
        "//modules/common/math",
        "//modules/common_msgs/basic_msgs:pnc_point_cc_proto",
        "//modules/common_msgs/config_msgs:vehicle_config_cc_proto",
        "//modules/control/control_component/controller_task_base:control_task",
        "//modules/control/control_component/controller_task_base/common:leadlag_controller",
        "//modules/control/control_component/controller_task_base/common:pid_controller",
        "//modules/control/control_component/controller_task_base/common:trajectory_analyzer",
        "//modules/control/controllers/lon_based_pid_controller/proto:lon_based_pid_controller_conf_proto",
    ],
)

apollo_package()

cpplint()
