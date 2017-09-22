load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_inc_library(
    name = "interface",
    hdrs = glob(["interface/*.h"]),
)

cc_library(
    name = "platform_common_lib",
    srcs = ["log.cc"],
    hdrs = glob(["*.h"]),
    deps = [
    ],
)

cc_binary(
    name = "log_test",
    srcs = ["tests/log_test.cc"],
    deps = [
        ":platform_common_lib",
    ],
)

cpplint()
