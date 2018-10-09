load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "transport_message_lib",
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
        "//framework/cybertron/transport/common:transport_common_lib",
    ],
)

cpplint()
