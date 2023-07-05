load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "plugin_manager",
    srcs = ["plugin_manager.cc"],
    hdrs = ["plugin_manager.h"],
    alwayslink = True,
    copts = ['-DMODULE_NAME=\\"dreamview\\"'],
    deps = [
        "//cyber",
        "//modules/dreamview/proto:dv_plugin_msg_cc_proto",
        "//modules/dreamview/proto:plugin_config_cc_proto",
        "//modules/dreamview/backend/handlers:websocket_handler",
        "//modules/common/util:util_tool",
        "//modules/dreamview/backend/common:dreamview_gflags",
    ],
)

cpplint()
