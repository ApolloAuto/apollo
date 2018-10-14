load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "timer_lib",
    srcs = glob([
        "*.cpp",
    ]),
    hdrs = glob([
        "*.h",
    ]),
    deps = [
          "//cybertron/common:common_lib",
          "//cybertron/croutine:croutine_lib",
          "//cybertron/data:data_lib",
          "//cybertron/event:event_lib",
          "//cybertron/scheduler:scheduler_lib",
          "//cybertron/task:task_lib",
          "//cybertron/time:time_lib",
    ],
)

cpplint()
