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
        "//cybertron:init_lib",
        "//cybertron/proto:cyber_config_cc_proto",
        "//cybertron/base:base_lib",
    ],
)

cpplint()
