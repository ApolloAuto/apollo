load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "qos_lib",
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
        "//framework/cybertron/proto:proto_lib",
    ],
)

cpplint()
