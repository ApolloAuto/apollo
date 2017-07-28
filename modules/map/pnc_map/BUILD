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
    ],
)

cc_test(
    name = "pnc_path_test",
    size = "small",
    srcs = [
        "path_test.cc",
    ],
    data = ["//modules/planning:planning_testdata"],
    deps = [
        ":path",
        "//modules/common:log",
        "//modules/common/time",
        "//modules/common/util",
        "@gtest//:main",
    ],
)

cpplint()
