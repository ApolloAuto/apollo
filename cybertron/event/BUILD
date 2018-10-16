load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "event_lib",
    srcs = glob([
        "*.cpp",
    ]),
    hdrs = glob([
        "*.h",
    ]),
    deps = [
          "//cybertron/base:bounded_queue",
          "//cybertron/base:macros",
          "//cybertron/base:wait_strategy",
          "//cybertron/common",
          "//cybertron:init_lib",
          "//cybertron/proto:perf_conf_cc_proto",
          "//cybertron/time:time",
    ],
)

cpplint()
