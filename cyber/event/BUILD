load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "perf_event_cache",
    srcs = [
        "perf_event_cache.cc",
    ],
    hdrs = [
        "perf_event_cache.h",
        "perf_event.h",
    ],
    deps = [
          "//cyber:init",
          "//cyber/base:bounded_queue",
          "//cyber/common",
          "//cyber/proto:perf_conf_cc_proto",
          "//cyber/time:time",
    ]
)

cpplint()
