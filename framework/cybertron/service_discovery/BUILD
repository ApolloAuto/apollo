load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "service_discovery_lib",
    srcs = glob([
        "*.cpp",
        "**/*.cpp",
    ]),
    hdrs = glob([
        "*.h",
        "**/*.h",
    ]),
    deps = [
        "//cybertron:init_lib",
        "//cybertron/common:common_lib",
        "//cybertron/proto:proto_lib",
        "//cybertron/base:base_lib",
        "//cybertron/message:message_lib",
        "//cybertron/time:time_lib",
        "//cybertron/event:event_lib",
        "//cybertron/transport/rtps:rtps_lib",
        "//cybertron/transport/qos:qos_lib",
    ],
)

cpplint()
