load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "blocker_lib",
    srcs = glob([
        "*.cpp",
    ]),
    hdrs = glob([
        "*.h",
    ]),
    deps = [
          "//cybertron:init_lib",
          "//cybertron/proto:proto_lib",
          "//cybertron/time:time_lib",
    ],
    copts = [
        "-Iframework",
    ],
)

cpplint()
