load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "class_loader_lib",
    srcs = glob([
        "*.cpp",
    ]),
    hdrs = glob([
        "*.h",
    ]),
    deps = [
          "//cybertron/common:common_lib",
    ],
    linkopts = [
        "-lPocoFoundation",
    ],
)

cpplint()
