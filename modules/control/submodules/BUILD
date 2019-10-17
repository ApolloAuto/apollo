load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

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
        "//modules/common",
        "//modules/common/adapters:adapter_gflags",
        "//modules/control/controller",
        "//modules/common/monitor_log",
        "//modules/common/time",
        "//modules/common/util",
        "//modules/common/status",
        "//modules/control/common:control_gflags",
        "//modules/canbus/proto:canbus_proto",
        "//modules/control/proto:mpc_controller_conf_proto",
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
