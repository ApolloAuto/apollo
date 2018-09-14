load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_test(
    name = "localization_msf_localization_integ_impl_test",
    size = "small",
    srcs = ["localization_integ_impl_test.cc"],
    deps = [
        "//modules/localization/msf/local_integ:localization_msf_local_integ",
        "@gtest//:main",
    ],
)

cpplint()
