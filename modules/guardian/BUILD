load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "guardian_lib",
    srcs = ["guardian_component.cc"],
    hdrs = ["guardian_component.h"],
    deps = [
        "//modules/common/util:message_util",
        "//modules/canbus/proto:canbus_proto",
        "//modules/control/proto:control_proto",
        "//modules/monitor/proto:system_status_proto",
        "//modules/guardian/proto:guardian_proto",
        "//modules/guardian/proto:guardian_conf_proto",
        "//framework:cybertron",
    ],
    copts = ['-DMODULE_NAME=\\"guardian\\"']
)

cc_binary(
    name = "libguardian.so",
    deps = [
        ":guardian_lib",
    ],
    linkopts = ["-shared"],
    linkstatic = False,
)

cpplint()
