load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "perf_event",
    srcs = [
        "perf_event.cc",
    ],
    hdrs = [
        "perf_event.h",
    ],
    deps = [
          "//cyber/base:bounded_queue",
          "//cyber/common",
    ],
)

cc_library(
    name = "perf_event_cache",
    srcs = [
        "perf_event_cache.cc",
    ],
    hdrs = [
        "perf_event_cache.h",
    ],
    deps = [
          "//cyber:init",
          "//cyber/event:perf_event",
          "//cyber/proto:perf_conf_cc_proto",
          "//cyber/time:time",
    ]
)

cpplint()
