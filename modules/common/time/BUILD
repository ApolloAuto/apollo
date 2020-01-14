load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "time",
    srcs = ["timer.cc"],
    hdrs = [
        "time.h",
        "time_util.h",
        "timer.h",
    ],
    deps = [
        "//cyber/time",
        "//modules/common/configs:config_gflags",
        "@com_google_absl//absl/time",
    ],
)

cc_test(
    name = "time_test",
    size = "small",
    srcs = ["time_test.cc"],
    deps = [
        ":time",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "timer_test",
    size = "small",
    srcs = ["timer_test.cc"],
    deps = [
        ":time",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
