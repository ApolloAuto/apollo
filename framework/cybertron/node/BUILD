load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "reader_base_lib",
    hdrs = glob([
        "reader_base.h",
    ]),
    copts = ["-Iframework"],
    deps = [
        "//framework/cybertron/transport:transport_lib",
        "//framework/cybertron/common:common_lib",
        "//framework/cybertron/event:event_lib",
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
    copts = ["-Iframework"],
    deps = [
        "//framework/cybertron/transport:transport_lib",
        "//framework/cybertron/blocker:blocker_lib",
        "//framework/cybertron/croutine:croutine_lib",
        "//framework/cybertron/data:data_lib",
        "//framework/cybertron/scheduler:scheduler_lib",
        "//framework/cybertron/service:service_lib",
    ],
)

cpplint()
