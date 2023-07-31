load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:apollo.bzl", "cyber_plugin_description")

package(default_visibility = ["//visibility:public"])



cyber_plugin_description(
    name = "plugin_lon_controller_description",
    description = ":plugins.xml",
    plugin = ":liblon_controller.so",
)

cc_binary(
    name = "liblon_controller.so",
    linkshared = True,
    linkstatic = True,
    copts = ["-DMODULE_NAME=\\\"control\\\""],
    deps = [":lon_controller_lib"],
)

cc_library(
    name = "lon_controller_lib",
    srcs = ["lon_controller.cc"],
    hdrs = ["lon_controller.h"],
    copts = ["-DMODULE_NAME=\\\"control\\\""],
    deps = [
        "//cyber",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/filters",
        "//modules/common/status",
        "//modules/control/control_component/common:control_gflags",
        "//modules/control/controllers/control_task_base:control_task",
        "//modules/control/controllers/control_task_base/common:interpolation_2d",
        "//modules/control/controllers/control_task_base/common:leadlag_controller",
        "//modules/control/controllers/control_task_base/common:pid_controller",
        "//modules/control/controllers/control_task_base/common:trajectory_analyzer",
        "//modules/control/controllers/control_task_base/proto:calibration_table_cc_proto",
        "//modules/common_msgs/control_msgs:control_cmd_cc_proto",
        "//modules/control/controllers/control_task_base/proto:gain_scheduler_conf_cc_proto",
        "//modules/common_msgs/control_msgs:input_debug_cc_proto",
        "//modules/control/controllers/lon_based_pid_controller/proto:lon_based_pid_controller_conf_cc_proto",
        "//modules/control/controllers/control_task_base/proto:mrac_conf_cc_proto",
        "//modules/common_msgs/control_msgs:control_pad_msg_cc_proto",
        "//modules/localization/common:localization_gflags",
        "@com_google_absl//:absl",
    ],
    alwayslink = True,
)

cc_test(
    name = "lon_controller_test",
    size = "small",
    srcs = ["lon_controller_test.cc"],
    copts = ["-fno-access-control"],
    data = ["longitudinal_controller_test"],
    deps = [
        ":lon_controller_lib",
        "//cyber",
        "//modules/common/util",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/control/control_component/common:control_gflags",
        "//modules/control/controllers/control_task_base:control_task",
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
