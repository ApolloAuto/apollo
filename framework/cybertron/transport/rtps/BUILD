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
    copts = [
        "-Iframework",
    ],
    deps = [
        "//framework/cybertron/common:common_lib",
        "//framework/cybertron/base:base_lib",
        "//framework/cybertron/proto:proto_lib",
        "//framework/cybertron/transport/common:transport_common_lib",
        "//framework/cybertron/transport/message:transport_message_lib",
        "//framework/cybertron/transport/qos:qos_lib",
        "@fastrtps//:fastrtps",
    ],
)

#cpplint()
