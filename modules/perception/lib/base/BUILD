load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "base",
    srcs = glob(["*.cc"]),
    hdrs = glob(["*.h"]),
    linkopts = [
        "-lboost_filesystem",
        "-lboost_system",
    ],
    deps = [
        "//modules/common:common",
        "//modules/common:log",
        "//external:gflags",
    ],
)

cpplint()
