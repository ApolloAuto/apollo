load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "rtps_lib",
    srcs = glob([
        "*.cpp",
    ]),
    hdrs = glob([
        "*.h",
    ]),
    deps = [
        "//cybertron/common:common_lib",
        "//cybertron/base:base_lib",
        "//cybertron/proto:proto_lib",
        "//cybertron/transport/common:transport_common_lib",
        "//cybertron/transport/message:transport_message_lib",
        "//cybertron/transport/qos:qos_lib",
        "@fastrtps//:fastrtps",
    ],
)

cpplint()
