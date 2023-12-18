load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_package", "apollo_cc_test")

package(default_visibility = ["//visibility:public"])

apollo_cc_library(
    name = "cyber_time",
    srcs = [
        "clock.cc",
        "duration.cc",
        "rate.cc",
        "time.cc",
    ],
    hdrs = [
        "clock.h",
        "duration.h",
        "rate.h",
        "time.h",
    ],
    deps = [
        "//cyber/common:cyber_common",
        "//cyber/base:cyber_base",
        "//cyber/proto:run_mode_conf_cc_proto",
    ],
)

apollo_cc_test(
    name = "time_test",
    size = "small",
    srcs = ["time_test.cc"],
    deps = [
        ":cyber_time",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "duration_test",
    size = "small",
    srcs = ["duration_test.cc"],
    deps = [
        ":cyber_time",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "clock_test",
    size = "small",
    srcs = ["clock_test.cc"],
    deps = [
        ":cyber_time",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_package()
cpplint()
