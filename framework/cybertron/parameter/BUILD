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
        "//cybertron/node:node_lib",
        "//cybertron/data:data_lib",
        "//cybertron/message:message_lib",
        "//cybertron/service:service_lib",
        "@fastrtps",
    ],
)

cpplint()
