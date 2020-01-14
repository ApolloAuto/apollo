load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "curve1d",
    hdrs = ["curve1d.h"],
)

cc_library(
    name = "polynomial_curve1d",
    hdrs = ["polynomial_curve1d.h"],
    deps = [
        ":curve1d",
    ],
)

cc_library(
    name = "quartic_polynomial_curve1d",
    srcs = ["quartic_polynomial_curve1d.cc"],
    hdrs = ["quartic_polynomial_curve1d.h"],
    deps = [
        ":polynomial_curve1d",
        "//cyber/common",
        "//modules/common/util:string_util",
        "@com_google_absl//absl/strings",
    ],
)

cc_test(
    name = "quartic_polynomial_curve1d_test",
    size = "small",
    srcs = ["quartic_polynomial_curve1d_test.cc"],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    deps = [
        ":cubic_polynomial_curve1d",
        ":quartic_polynomial_curve1d",
        ":quintic_polynomial_curve1d",
        "//cyber/common:log",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "quintic_polynomial_curve1d",
    srcs = ["quintic_polynomial_curve1d.cc"],
    hdrs = ["quintic_polynomial_curve1d.h"],
    deps = [
        ":polynomial_curve1d",
        "//cyber/common",
        "//modules/common/util:string_util",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "cubic_polynomial_curve1d",
    srcs = ["cubic_polynomial_curve1d.cc"],
    hdrs = ["cubic_polynomial_curve1d.h"],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    deps = [
        ":polynomial_curve1d",
        ":quartic_polynomial_curve1d",
        "//cyber/common:log",
        "@com_google_absl//absl/strings",
    ],
)

cc_test(
    name = "cubic_polynomial_curve1d_test",
    size = "small",
    srcs = ["cubic_polynomial_curve1d_test.cc"],
    deps = [
        ":cubic_polynomial_curve1d",
        ":quartic_polynomial_curve1d",
        "//cyber/common:log",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "quintic_spiral_path",
    srcs = ["quintic_spiral_path.cc"],
    hdrs = [
        "quintic_spiral_path.h",
        "quintic_spiral_path_with_derivation.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    deps = [
        ":quintic_polynomial_curve1d",
        "//cyber/common:log",
        "//modules/common/math",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "piecewise_quintic_spiral_path",
    srcs = ["piecewise_quintic_spiral_path.cc"],
    hdrs = ["piecewise_quintic_spiral_path.h"],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    deps = [
        ":quintic_spiral_path",
        "//cyber/common:log",
        "//modules/common/math",
        "@com_google_absl//absl/strings",
    ],
)

cc_test(
    name = "quintic_polynomial_curve1d_test",
    size = "small",
    srcs = ["quintic_polynomial_curve1d_test.cc"],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    deps = [
        ":quartic_polynomial_curve1d",
        ":quintic_polynomial_curve1d",
        "//cyber/common:log",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "piecewise_quintic_spiral_path_test",
    size = "small",
    srcs = ["piecewise_quintic_spiral_path_test.cc"],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    deps = [
        ":piecewise_quintic_spiral_path",
        "//cyber/common:log",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
