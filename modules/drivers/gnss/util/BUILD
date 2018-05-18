load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "gnss_util",
    srcs = [
        "utils.cc",
    ],
    hdrs = [
        "macros.h",
        "utils.h",
        "time_conversion.h"
    ],
    deps = [
        "//modules/drivers/gnss/proto:gnss_proto",
    ],
)

cpplint()
