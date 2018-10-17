load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "transport_message_lib",
    srcs = glob([
        "*.cc",
    ]),
    hdrs = glob([
        "*.h",
    ]),
    deps = [
        "//cybertron/base:signal",
        "//cybertron/common:common",
        "//cybertron/message:raw_message",
        "//cybertron/transport/common:transport_common",
    ],
)

cpplint()
