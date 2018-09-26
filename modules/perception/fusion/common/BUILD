load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "base_filter",
    hdrs = [
        "base_filter.h",
    ],
    deps = [
        "@eigen",
    ],
)

cc_library(
    name = "camera_util",
    srcs = [
        "camera_util.cc",
    ],
    hdrs = [
        "camera_util.h",
    ],
    deps = [
        "//modules/perception/base:camera",
        "//modules/perception/common/geometry:common",
        "//modules/perception/fusion/base:sensor",
    ],
)

cc_library(
    name = "dst_evidence",
    srcs = [
        "dst_evidence.cc",
    ],
    hdrs = [
        "dst_evidence.h",
    ],
    deps = [
        "//framework:cybertron",
    ],
)

cc_library(
    name = "information_filter",
    srcs = [
        "information_filter.cc",
    ],
    hdrs = [
        "information_filter.h",
    ],
    deps = [
        ":base_filter",
    ],
)

cc_library(
    name = "kalman_filter",
    srcs = [
        "kalman_filter.cc",
    ],
    hdrs = [
        "kalman_filter.h",
    ],
    deps = [
        ":base_filter",
        "//framework:cybertron",
    ],
)

cpplint()
