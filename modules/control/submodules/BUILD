load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

PREPROCESS_SUB_COPTS = ["-DMODULE_NAME=\\\"preprocessor_submodule\\\""]

cc_library(
    name = "control_module_lib",
    deps = [
        ":lat_lon_controller_submodule_lib",
        ":mpc_controller_submodule_lib",
        ":postprocessor_submodule_lib",
        ":preprocessor_submodule_lib",
    ],
)

cc_binary(
    name = "control_module.so",
    linkshared = True,
    linkstatic = False,
    deps = [":control_module_lib"],
)

cc_library(
    name = "preprocessor_submodule_lib",
    srcs = ["preprocessor_submodule.cc"],
    hdrs = ["preprocessor_submodule.h"],
    copts = PREPROCESS_SUB_COPTS,
    deps = [
        "//cyber",
        "//cyber/time:clock",
        "//modules/canbus/proto:chassis_cc_proto",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/latency_recorder",
        "//modules/common/monitor_log",
        "//modules/common/status",
        "//modules/common/util",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/control/common:control_gflags",
        "//modules/control/common:dependency_injector",
        "//modules/control/proto:control_cmd_cc_proto",
        "//modules/control/proto:control_common_conf_cc_proto",
        "//modules/control/proto:pad_msg_cc_proto",
        "//modules/control/proto:preprocessor_cc_proto",
        "//modules/localization/proto:localization_cc_proto",
        "//modules/planning/proto:planning_cc_proto",
    ],
)

cc_binary(
    name = "preprocessor_submodule.so",
    linkshared = True,
    linkstatic = False,
    deps = [":preprocessor_submodule_lib"],
)

cc_library(
    name = "mpc_controller_submodule_lib",
    srcs = ["mpc_controller_submodule.cc"],
    hdrs = ["mpc_controller_submodule.h"],
    copts = PREPROCESS_SUB_COPTS,
    deps = [
        "//cyber",
        "//cyber/time:clock",
        "//modules/canbus/proto:chassis_cc_proto",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/latency_recorder",
        "//modules/common/monitor_log",
        "//modules/common/status",
        "//modules/common/util",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/control/common:control_gflags",
        "//modules/control/controller",
        "//modules/control/proto:calibration_table_cc_proto",
        "//modules/control/proto:control_cmd_cc_proto",
        "//modules/control/proto:control_conf_cc_proto",
        "//modules/control/proto:pad_msg_cc_proto",
        "//modules/control/proto:preprocessor_cc_proto",
        "//modules/localization/proto:localization_cc_proto",
        "//modules/planning/proto:planning_cc_proto",
        "@com_github_gflags_gflags//:gflags",
    ],
    alwayslink = True,
)

cc_binary(
    name = "mpc_controller_submodule.so",
    linkshared = True,
    linkstatic = False,
    deps = [":mpc_controller_submodule_lib"],
)

cc_library(
    name = "lat_lon_controller_submodule_lib",
    srcs = ["lat_lon_controller_submodule.cc"],
    hdrs = ["lat_lon_controller_submodule.h"],
    copts = PREPROCESS_SUB_COPTS,
    deps = [
        "//cyber",
        "//cyber/time:clock",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/latency_recorder",
        "//modules/common/monitor_log",
        "//modules/common/status",
        "//modules/common/util",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/control/common:control_gflags",
        "//modules/control/controller",
        "//modules/control/proto:local_view_cc_proto",
        "//modules/control/proto:preprocessor_cc_proto",
        "@com_github_gflags_gflags//:gflags",
    ],
    alwayslink = True,
)

cc_binary(
    name = "lat_lon_controller_submodule.so",
    linkshared = True,
    linkstatic = False,
    deps = [":lat_lon_controller_submodule_lib"],
)

cc_library(
    name = "postprocessor_submodule_lib",
    srcs = ["postprocessor_submodule.cc"],
    hdrs = ["postprocessor_submodule.h"],
    copts = PREPROCESS_SUB_COPTS,
    deps = [
        "//cyber",
        "//cyber/time:clock",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/latency_recorder",
        "//modules/common/monitor_log",
        "//modules/common/status",
        "//modules/common/util",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/control/common:control_gflags",
        "//modules/control/controller",
        "//modules/control/proto:calibration_table_cc_proto",
        "//modules/control/proto:control_cmd_cc_proto",
        "//modules/control/proto:control_common_conf_cc_proto",
        "//modules/control/proto:control_conf_cc_proto",
        "//modules/control/proto:pad_msg_cc_proto",
        "//modules/control/proto:preprocessor_cc_proto",
        "//modules/localization/proto:localization_cc_proto",
        "@com_github_gflags_gflags//:gflags",
    ],
    alwayslink = True,
)

cc_binary(
    name = "postprocessor_submodule.so",
    linkshared = True,
    linkstatic = False,
    deps = [":postprocessor_submodule_lib"],
)

cpplint()
