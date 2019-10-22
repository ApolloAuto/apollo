load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "pre_process_submodule_lib",
    srcs = ["pre_process_submodule.cc"],
    hdrs = [
        "pre_process_submodule.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"mpc_controller_submodule\\\"",
    ],
    deps = [
        "//cyber",
        "//external:gflags",
        "//modules/canbus/proto:canbus_proto",
        "//modules/common",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/monitor_log",
        "//modules/common/status",
        "//modules/common/time",
        "//modules/common/util",
        "//modules/control/common:control_gflags",
        "//modules/control/proto:control_command_proto",
        "//modules/control/proto:control_common_conf_proto",
        "//modules/localization/proto:localization_proto",
        "//modules/planning/proto:planning_proto",
    ],
)

cc_binary(
    name = "pre_process_submodule.so",
    linkshared = True,
    linkstatic = False,
    deps = [":pre_process_submodule_lib"],
)

cc_library(
    name = "mpc_controller_submodule_lib",
    srcs = ["mpc_controller_submodule.cc"],
    hdrs = [
        "mpc_controller_submodule.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"mpc_controller_submodule\\\"",
    ],
    deps = [
        "//cyber",
        "//external:gflags",
        "//modules/canbus/proto:canbus_proto",
        "//modules/common",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/monitor_log",
        "//modules/common/status",
        "//modules/common/time",
        "//modules/common/util",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/control/common:control_gflags",
        "//modules/control/controller",
        "//modules/control/proto:control_proto",
        "//modules/localization/proto:localization_proto",
        "//modules/planning/proto:planning_proto",
    ],
)

cc_binary(
    name = "mpc_controller_submodule.so",
    linkshared = True,
    linkstatic = False,
    deps = [":mpc_controller_submodule_lib"],
)

cpplint()
