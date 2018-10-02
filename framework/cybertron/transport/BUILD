load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "transport_lib",
    srcs = glob([
        "*.cpp",
        "**/*.cpp",
    ]),
    hdrs = glob([
        "*.h",
        "**/*.h",
    ]),
    copts = [
        "-Iframework",
    ],
    deps = [
        "//framework/cybertron/base:base_lib",
        "//framework/cybertron/common:common_lib",
        "//framework/cybertron/event:event_lib",
        "//framework/cybertron/message:message_lib",
        "//framework/cybertron/proto:proto_lib",
        "//framework/cybertron/service_discovery:service_discovery_lib",
        "//framework/cybertron/transport/rtps:rtps_lib",
        "//framework/cybertron/transport/common:transport_common_lib",
        "//framework/cybertron/transport/message:transport_message_lib",
        "//framework/cybertron/time:time_lib",
        "//framework/cybertron/timer:timer_lib",
    ],
)

#cpplint()
