load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "local_path",
    srcs = [
        "local_path.cc",
    ],
    hdrs = [
        "local_path.h",
    ],
    deps = [
        "//modules/common/math",
        "//modules/common:log",
        "//modules/common/proto:pnc_point_proto",
    ],
)


cpplint()
