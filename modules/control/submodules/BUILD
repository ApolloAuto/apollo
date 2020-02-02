load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

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
    copts = [
        "-DMODULE_NAME=\\\"preprocessor_submodule\\\"",
    ],
    deps = [
        "//cyber",
        "//modules/canbus/proto:canbus_proto",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/latency_recorder",
        "//modules/common/monitor_log",
        "//modules/common/status",
        "//modules/common/time",
        "//modules/common/util",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/control/common:control_gflags",
        "//modules/control/proto:control_command_proto",
        "//modules/control/proto:control_common_conf_proto",
        "//modules/control/proto:preprocessor_proto",
        "//modules/localization/proto:localization_proto",
        "//modules/planning/proto:planning_proto",
        "@com_github_gflags_gflags//:gflags",
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
    copts = [
        "-DMODULE_NAME=\\\"mpc_controller_submodule\\\"",
    ],
    deps = [
        "//cyber",
        "//modules/canbus/proto:canbus_proto",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/latency_recorder",
        "//modules/common/monitor_log",
        "//modules/common/status",
        "//modules/common/time",
        "//modules/common/util",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/control/common:control_gflags",
        "//modules/control/controller",
        "//modules/control/proto:calibration_table_proto",
        "//modules/control/proto:control_common_conf_proto",
        "//modules/control/proto:control_proto",
        "//modules/control/proto:preprocessor_proto",
        "//modules/localization/proto:localization_proto",
        "//modules/planning/proto:planning_proto",
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
    copts = [
        "-DMODULE_NAME=\\\"lat_lon_controller_submodule\\\"",
    ],
    deps = [
        "//cyber",
        "//modules/canbus/proto:canbus_proto",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/latency_recorder",
        "//modules/common/monitor_log",
        "//modules/common/status",
        "//modules/common/time",
        "//modules/common/util",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/control/common:control_gflags",
        "//modules/control/controller",
        "//modules/control/proto:control_common_conf_proto",
        "//modules/control/proto:control_proto",
        "//modules/control/proto:preprocessor_proto",
        "//modules/localization/proto:localization_proto",
        "//modules/planning/proto:planning_proto",
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
    copts = [
        "-DMODULE_NAME=\\\"postprocessor_submodule\\\"",
    ],
    deps = [
        "//cyber",
        "//modules/canbus/proto:canbus_proto",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/latency_recorder",
        "//modules/common/monitor_log",
        "//modules/common/status",
        "//modules/common/time",
        "//modules/common/util",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/control/common:control_gflags",
        "//modules/control/controller",
        "//modules/control/proto:control_common_conf_proto",
        "//modules/control/proto:control_proto",
        "//modules/control/proto:preprocessor_proto",
        "//modules/localization/proto:localization_proto",
        "//modules/planning/proto:planning_proto",
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
