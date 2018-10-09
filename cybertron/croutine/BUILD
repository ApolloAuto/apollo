load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "croutine_lib",
    srcs = glob([
        "*.cpp",
        "*.S",
    ]),
    hdrs = glob([
        "*.h",
    ]),
    deps = [
          "//cybertron:init_lib",
          "//cybertron/common:common_lib",
          "//cybertron/event:event_lib",
          "//cybertron/proto:proto_lib",
          "//cybertron/time:time_lib",
    ],
    copts = [
        "-Iframework",
    ],
)

cpplint()
