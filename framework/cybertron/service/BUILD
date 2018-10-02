load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "service_lib",
    srcs = glob([
        "*.cpp",
    ]),
    hdrs = glob([
        "*.h",
    ]),
    deps = [
          "//framework/cybertron:init_lib",
          "//framework/cybertron/proto:proto_lib",
          "//framework/cybertron/time:time_lib",
    ],
    copts = [
        "-Iframework",
    ],
)

#cpplint()
