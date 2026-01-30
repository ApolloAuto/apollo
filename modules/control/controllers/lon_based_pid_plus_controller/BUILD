load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package", "apollo_cc_test", "apollo_plugin")

package(default_visibility = ["//visibility:public"])

CONTROL_COPTS = ['-DMODULE_NAME=\\"control\\"']

apollo_plugin(
    name = "liblon_plus_controller.so",
    srcs = [
        "lon_plus_controller.cc",
    ],
    hdrs = [
        "lon_plus_controller.h",
    ],
    description = ":plugins.xml",
    copts = CONTROL_COPTS,
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/filters",
        "//modules/common/status",
        "//modules/control/control_component/common:control_gflags",
        "//modules/control/control_component/controller_task_base/common:controllers_control_task_base_common",
        "//modules/control/control_component/proto:calibration_table_cc_proto",
        "//modules/control/control_task_base_extend:control_task_extend",
        "//modules/control/control_task_base_extend/common:controllers_control_task_base_extend_common",
        "//modules/control/control_task_base_extend/proto:params_pipeline_proto",
        "//modules/common_msgs/chassis_msgs:chassis_proto",
        "//modules/common_msgs/control_msgs:control_cmd_cc_proto",
        "//modules/control/control_component/proto:gain_scheduler_conf_cc_proto",
        "//modules/common_msgs/control_msgs:input_debug_cc_proto",
        "//modules/control/controllers/lon_based_pid_plus_controller/proto:lon_based_pid_plus_controller_conf_cc_proto",
        "//modules/control/control_component/proto:mrac_conf_cc_proto",
        "//modules/common_msgs/control_msgs:control_pad_msg_cc_proto",
        "@com_google_absl//:absl",
    ],
)

filegroup(
    name = "longitudinal_controller_test",
    srcs = glob([
        "longitudinal_controller_test/**",
    ]) + glob(["conf/*"]),
)

apollo_package(enable_source=False)
cpplint()
