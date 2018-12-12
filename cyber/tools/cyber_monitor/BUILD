load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "cyber_monitor",
    srcs = [
        "cyber_topology_message.cc",
        "general_channel_message.cc",
        "general_message.cc",
        "general_message_base.cc",
        "main.cc",
        "renderable_message.cc",
        "screen.cc",
    ],
    deps = [
        "cyber_topology_message",
        "general_channel_message",
        "screen",
        "//cyber/service_discovery:topology_manager",
        "//cyber:init",
    ],
    linkopts = [
        "-pthread",
        "-lncurses"
    ],
)

cc_library(
    name = "cyber_topology_message",
    hdrs = [ "cyber_topology_message.h", ],
    deps = [
        "renderable_message",
    ],
)

cc_library(
    name = "general_channel_message",
    hdrs = [ "general_channel_message.h", ],
    deps = [
        "general_message_base",
        "general_message",
        "screen",
        "//cyber/message:raw_message"
    ],
)

cc_library(
    name = "general_message",
    hdrs = [ "general_message.h", ],
    deps = [
        "general_message_base",
        "//cyber/message:raw_message",
        "//cyber:cyber",
    ],
)

cc_library(
    name = "general_message_base",
    hdrs = [ "general_message_base.h", ],
    deps = [
        "renderable_message",
    ],
)

cc_library(
    name = "renderable_message",
    hdrs = [ "renderable_message.h", ],
    deps = [
        "screen",
    ],
)

cc_library(
    name = "screen",
    hdrs = [ "screen.h", ],
)

#cpplint()
