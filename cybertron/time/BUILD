load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "time_lib",
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
          "//cybertron/common:common_lib",
          "//cybertron/proto:proto_lib",
    ],
)

cpplint()
