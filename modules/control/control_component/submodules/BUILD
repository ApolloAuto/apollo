load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package", "apollo_cc_binary", "apollo_cc_library")

package(default_visibility = ["//visibility:public"])

PREPROCESS_SUB_COPTS = ["-DMODULE_NAME=\\\"preprocessor_submodule\\\""]

apollo_cc_library(
    name = "control_module_lib",
    deps = [
        ":lat_lon_controller_submodule_lib",
        ":mpc_controller_submodule_lib",
        ":postprocessor_submodule_lib",
        ":preprocessor_submodule_lib",
    ],
)

apollo_cc_binary(
    name = "control_module.so",
    linkshared = True,
    linkstatic = True,
    deps = [":control_module_lib"],
)

apollo_cc_library(
    name = "preprocessor_submodule_lib",
    srcs = ["preprocessor_submodule.cc"],
    hdrs = ["preprocessor_submodule.h"],
    copts = PREPROCESS_SUB_COPTS,
    deps = [
        "//cyber",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/latency_recorder",
        "//modules/common/monitor_log",
        "//modules/common/status",
        "//modules/common/util:common_util",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/control/control_component/common:control_gflags",
        "//modules/control/control_component/controller_task_base/common:dependency_injector",
        "//modules/common_msgs/control_msgs:control_cmd_cc_proto",
        "//modules/common_msgs/control_msgs:control_pad_msg_cc_proto",
        "//modules/control/control_component/proto:preprocessor_cc_proto",
        "//modules/common_msgs/localization_msgs:localization_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
    ],
)

apollo_cc_binary(
    name = "preprocessor_submodule.so",
    linkshared = True,
    linkstatic = True,
    deps = [":preprocessor_submodule_lib"],
)

apollo_cc_library(
    name = "mpc_controller_submodule_lib",
    srcs = ["mpc_controller_submodule.cc"],
    hdrs = ["mpc_controller_submodule.h"],
    copts = PREPROCESS_SUB_COPTS,
    deps = [
        "//cyber",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/latency_recorder",
        "//modules/common/monitor_log",
        "//modules/common/status",
        "//modules/common/util:common_util",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/control/control_component/common:control_gflags",
        "//modules/control/control_component/controller_task_base:control_task",
        "//modules/control/control_component/proto:calibration_table_cc_proto",
        "//modules/common_msgs/control_msgs:control_cmd_cc_proto",
        "//modules/common_msgs/control_msgs:control_pad_msg_cc_proto",
        "//modules/control/control_component/proto:preprocessor_cc_proto",
        "//modules/common_msgs/localization_msgs:localization_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "@com_github_gflags_gflags//:gflags",
    ],
)

apollo_cc_binary(
    name = "mpc_controller_submodule.so",
    linkshared = True,
    linkstatic = True,
    deps = [":mpc_controller_submodule_lib"],
)

apollo_cc_library(
    name = "lat_lon_controller_submodule_lib",
    srcs = ["lat_lon_controller_submodule.cc"],
    hdrs = ["lat_lon_controller_submodule.h"],
    copts = PREPROCESS_SUB_COPTS,
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/latency_recorder",
        "//modules/common/monitor_log",
        "//modules/common/status",
        "//modules/common/util:common_util",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/common_msgs/control_msgs:control_cmd_cc_proto",
        "//modules/control/control_component/common:control_gflags",
        "//modules/control/control_component/controller_task_base:control_task",
        "//modules/control/control_component/proto:local_view_cc_proto",
        "//modules/control/control_component/proto:preprocessor_cc_proto",
    ],
)

apollo_cc_binary(
    name = "lat_lon_controller_submodule.so",
    linkshared = True,
    linkstatic = True,
    deps = [":lat_lon_controller_submodule_lib"],
)

apollo_cc_library(
    name = "postprocessor_submodule_lib",
    srcs = ["postprocessor_submodule.cc"],
    hdrs = ["postprocessor_submodule.h"],
    copts = PREPROCESS_SUB_COPTS,
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/latency_recorder",
        "//modules/common/monitor_log",
        "//modules/common/status",
        "//modules/common/util:common_util",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/control/control_component/common:control_gflags",
        "//modules/control/control_component/controller_task_base:control_task",
        "//modules/control/control_component/proto:calibration_table_cc_proto",
        "//modules/common_msgs/control_msgs:control_cmd_cc_proto",
        "//modules/common_msgs/control_msgs:control_pad_msg_cc_proto",
        "//modules/control/control_component/proto:preprocessor_cc_proto",
        "//modules/common_msgs/localization_msgs:localization_cc_proto",
        "@com_github_gflags_gflags//:gflags",
    ],
)

apollo_cc_binary(
    name = "postprocessor_submodule.so",
    linkshared = True,
    linkstatic = True,
    deps = [":postprocessor_submodule_lib"],
)

apollo_package()
cpplint()
