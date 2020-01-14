load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_test(
    name = "conti_ars_preprocessor_test",
    size = "small",
    srcs = ["conti_ars_preprocessor_test.cc"],
    deps = [
        ":conti_ars_preprocessor",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "conti_ars_preprocessor",
    srcs = ["conti_ars_preprocessor.cc"],
    hdrs = ["conti_ars_preprocessor.h"],
    deps = [
        "//modules/drivers/proto:sensor_proto",
        "//modules/perception/base",
        "//modules/perception/lib/config_manager",
        "//modules/perception/lib/registerer",
        "//modules/perception/lib/utils",
        "//modules/perception/radar/common:types",
        "//modules/perception/radar/lib/interface:base_preprocessor",
        "@eigen",
    ],
    alwayslink = True,
)

cpplint()
