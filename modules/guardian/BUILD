load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "guardian_component",
    srcs = ["guardian_component.cc"],
    hdrs = ["guardian_component.h"],
    copts = ['-DMODULE_NAME=\\"guardian\\"'],
    deps = [
        "//cyber",
        "//modules/canbus/proto:chassis_cc_proto",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/util:message_util",
        "//modules/control/proto:control_cmd_cc_proto",
        "//modules/guardian/proto:guardian_cc_proto",
        "//modules/guardian/proto:guardian_conf_cc_proto",
        "//modules/monitor/proto:system_status_cc_proto",
    ],
)

cc_binary(
    name = "libguardian.so",
    linkshared = True,
    linkstatic = False,
    deps = [
        ":guardian_component",
    ],
)

cpplint()
