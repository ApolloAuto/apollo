load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "pnc_path",
    srcs = [
        "path.cc",
    ],
    hdrs = [
        "path.h",
    ],
    deps = [
        "//modules/common/math",
        "//modules/map/hdmap",
        "//modules/map/proto:map_proto",
    ],
)

cpplint()
