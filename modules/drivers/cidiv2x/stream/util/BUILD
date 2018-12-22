load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "stream_util",
    srcs = [
        "utils.cc",
    ],
    hdrs = [
        "macros.h",
        "utils.h",
    ],
    deps = [
        "//modules/drivers/cidiv2x/stream/proto:stream_proto",
    ],
)

cpplint()
