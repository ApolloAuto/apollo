load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "path",
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
        "//modules/routing/proto:routing_proto",
    ],
)

cc_library(
    name = "pnc_map",
    srcs = [
        "pnc_map.cc",
    ],
    hdrs = [
        "pnc_map.h",
    ],
    deps = [
        "//modules/map/hdmap",
        "//modules/map/pnc_map:path",
    ],
)

cc_test(
    name = "pnc_path_test",
    size = "small",
    srcs = [
        "path_test.cc",
    ],
    deps = [
        ":path",
        "//modules/common:log",
        "//modules/common/time",
        "//modules/common/util",
        "@gtest//:main",
    ],
)

cpplint()
