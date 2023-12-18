load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_package", "apollo_cc_test")

package(default_visibility = ["//visibility:public"])

apollo_cc_test(
    name = "sysmo_test",
    size = "small",
    srcs = ["sysmo_test.cc"],
    deps = [
        "//cyber",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_library(
    name = "cyber_sysmo",
    srcs = ["sysmo.cc"],
    hdrs = ["sysmo.h"],
    deps = [
        "//cyber/scheduler:cyber_scheduler",
    ],
)

apollo_package()
cpplint()
