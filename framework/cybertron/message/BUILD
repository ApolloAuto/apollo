load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "message_lib",
    srcs = glob([
        "*.cpp",
    ]),
    hdrs = glob([
        "*.h",
    ]),
    deps = [
        "//framework/cybertron/common:common_lib",
        "//framework/cybertron/proto:proto_lib",
    ],
    copts = [
        "-Iframework",
    ],
)

#cpplint()
