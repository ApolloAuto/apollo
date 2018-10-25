load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "cyber_monitor",
    srcs = [
        "channel_msg_factory.cc",
        "cyber_channel_message.cc",
        "cyber_topology_message.cc",
        "general_channel_message.cc",
        "general_message.cc",
        "general_message_base.cc",
        "main.cc",
        "renderable_message.cc",
        "screen.cc",
    ],
    deps = [
        "channel_msg_factory",
        "cyber_channel_message",
        "cyber_topology_message",
        "general_channel_message",
        "screen",
        "//cyber:init",
        "//cyber/service_discovery:topology_manager",
    ],
    linkopts = [
        "-pthread",
        "-lncurses"
    ],
)

cc_library(
    name = "channel_msg_factory",
    hdrs = [ "channel_msg_factory.h", ],
)

cc_library(
    name = "cyber_channel_message",
    hdrs = [ "cyber_channel_message.h", ],
    deps = [
        "general_message",
        "renderable_message",
        "//cyber:cyber",
        "//cyber/time:time",
        "//cyber/time:duration",
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
        "//cyber:cyber",
        "//cyber/message:raw_message",
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
