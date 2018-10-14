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
    deps = [
        "//cybertron/base:base_lib",
        "//cybertron/common:common_lib",
        "//cybertron/event:event_lib",
        "//cybertron/message:message_lib",
        "//cybertron/proto:qos_profile_cc_proto",
        "//cybertron/proto:role_attributes_cc_proto",
        "//cybertron/proto:transport_conf_cc_proto",
        "//cybertron/service_discovery:service_discovery_lib",
        "//cybertron/transport/rtps:rtps_lib",
        "//cybertron/transport/common:transport_common_lib",
        "//cybertron/transport/message:transport_message_lib",
        "//cybertron/time:time_lib",
        "//cybertron/timer:timer_lib",
    ],
)

cpplint()
