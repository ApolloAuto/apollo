load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "libguardian.so",
    srcs = [
        "guardian_component.cc",
        "guardian_component.h",
    ],
    deps = [
        "//modules/common/util:message_util",
        "//modules/canbus/proto:canbus_proto",
        "//modules/control/proto:control_proto",
        "//modules/monitor/proto:system_status_proto",
        "//modules/guardian/proto:guardian_proto",
        "//modules/guardian/proto:guardian_conf_proto",
        "//framework:cybertron",
    ],
    copts = ['-DMODULE_NAME=\\"guardian\\"'],
    linkshared = True,
)

cpplint()
