load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "log",
    srcs = ["log.cc"],
    hdrs = glob(["*.h"]),
)

cc_binary(
    name = "log_test",
    srcs = ["log_test.cc"],
    deps = [
        ":log",
    ],
)

cpplint()
