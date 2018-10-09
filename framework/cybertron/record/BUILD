load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "record_lib",
    srcs = glob([
        "*.cpp",
    ]),
    hdrs = glob([
        "*.h",
    ]),
    deps = [
        "//framework/cybertron:init_lib",
        "//framework/cybertron/common:common_lib",
        "//framework/cybertron/message:message_lib",
        "//framework/cybertron/proto:proto_lib",
    ],
    copts = ["-Iframework"],
)

cpplint()
