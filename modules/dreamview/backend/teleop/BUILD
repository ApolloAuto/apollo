load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "teleop",
    srcs = select({
        "//modules/dreamview/backend:teleop": [
            "teleop.cc",
        ],
        "//conditions:default": [
        ],
    }),
    hdrs = select({
        "//modules/dreamview/backend:teleop": [
            "teleop.h",
        ],
        "//conditions:default": [
        ],
    }),
    copts = ['-DMODULE_NAME=\\"dreamview\\"'],
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/monitor_log",
        "//modules/common/proto:drive_event_proto",
        "//modules/common/util",
        "//modules/dreamview/backend/common:dreamview_gflags",
        "//modules/dreamview/backend/handlers:websocket_handler",
        "//modules/planning/proto:planning_proto",
        "//third_party/json",
    ] + select({
        "//modules/dreamview/backend:teleop": [
            "//modules/teleop/modem/proto:modem_proto",
            "//modules/teleop/daemon/proto:daemon_proto",
        ],
        "//conditions:default": [
        ],
    }),
)

cc_test(
    name = "teleop_test",
    size = "small",
    srcs = select({
        "//modules/dreamview/backend:teleop": [
            "teleop_test.cc",
        ],
        "//conditions:default": [
        ],
    }),
    data = [
        "//modules/dreamview/backend/testdata",
    ],
    deps = [
        ":teleop",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
