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
    copts = [
        "-Iframework",
    ],
    deps = [
          "//framework/cybertron/common:common_lib",
          "//framework/cybertron/proto:proto_lib",
          "//framework/cybertron/time:time_lib",
    ],
)

#cpplint()
