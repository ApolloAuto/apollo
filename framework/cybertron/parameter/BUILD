load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "parameter_lib",
    srcs = glob([
        "*.cpp",
    ]),
    hdrs = glob([
        "*.h",
    ]),
    deps = [
        "//framework/cybertron/node:node_lib",
        "//framework/cybertron/data:data_lib",
        "//framework/cybertron/message:message_lib",
        "//framework/cybertron/service:service_lib",
        "@fastrtps",
    ],
    copts = ["-Iframework"],
)

cpplint()
