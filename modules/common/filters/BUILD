load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_package", "apollo_cc_test")

apollo_cc_library(
    name = "filters",
    hdrs = [
        "digital_filter.h",
        "digital_filter_coefficients.h",
        "mean_filter.h",
    ],
    srcs = [
        "digital_filter.cc",
        "digital_filter_coefficients.cc",
        "mean_filter.cc",
    ],
    deps = [
        "//cyber",
    ],
)

apollo_cc_test(
    name = "digital_filter_test",
    size = "small",
    srcs = ["digital_filter_test.cc"],
    linkstatic = True,
    deps = [
        ":filters",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "mean_filter_test",
    size = "small",
    srcs = ["mean_filter_test.cc"],
    linkstatic = True,
    deps = [
        ":filters",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "digital_filter_coefficients_test",
    size = "small",
    srcs = ["digital_filter_coefficients_test.cc"],
    deps = [
        ":filters",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_package()

cpplint()
