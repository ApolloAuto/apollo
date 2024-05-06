load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package", "apollo_cc_binary")

package(default_visibility = ["//visibility:public"])


apollo_cc_binary(
    name = "cyber_monitor",
    srcs = [
        "cyber_topology_message.cc",
        "general_channel_message.cc",
        "general_message.cc",
        "general_message_base.cc",
        "main.cc",
        "renderable_message.cc",
        "screen.cc",
        "cyber_topology_message.h",
        "general_channel_message.h",
        "general_message.h",
        "general_message_base.h",
        "renderable_message.h",
        "screen.h",
    ],
    linkopts = ["-pthread"],
    deps = [
        "//cyber",
        "@ncurses5",
    ],
)

apollo_package()
cpplint()
