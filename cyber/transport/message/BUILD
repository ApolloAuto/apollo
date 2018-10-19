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
        "//cyber/base:signal",
        "//cyber/common:common",
        "//cyber/message:raw_message",
        "//cyber/transport/common:transport_common",
    ],
)

cpplint()
