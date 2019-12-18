load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "guardian_component",
    srcs = ["guardian_component.cc"],
    hdrs = ["guardian_component.h"],
    copts = ['-DMODULE_NAME=\\"guardian\\"'],
    deps = [
        "//cyber",
        "//modules/canbus/proto:canbus_proto",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/util:message_util",
        "//modules/control/proto:control_proto",
        "//modules/guardian/proto:guardian_conf_proto",
        "//modules/guardian/proto:guardian_proto",
        "//modules/monitor/proto:system_status_proto",
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
