load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "reader_base_lib",
    hdrs = glob([
        "reader_base.h",
    ]),
    deps = [
        "//cybertron/transport:transport_lib",
        "//cybertron/common:common_lib",
        "//cybertron/event:event_lib",
    ],
)

cc_library(
    name = "node_lib",
    srcs = glob([
        "*.cpp",
    ]),
    hdrs = glob([
        "*.h",
    ]),
    deps = [
        "//cybertron/transport:transport_lib",
        "//cybertron/blocker:blocker_lib",
        "//cybertron/croutine:croutine_lib",
        "//cybertron/data:data_lib",
        "//cybertron/scheduler:scheduler_lib",
        "//cybertron/service:service_lib",
    ],
)

cpplint()
