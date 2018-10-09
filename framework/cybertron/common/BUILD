load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "common_lib",
    srcs = glob([
          "*.cpp",
    ]),
    hdrs = glob([
          "*.h",
    ]),
    deps = [
        "//framework/cybertron:init_lib",
        "//framework/cybertron/proto:proto_lib",
        "//framework/cybertron/base:base_lib",
    ],
    copts = ["-Iframework"],
)

cpplint()
