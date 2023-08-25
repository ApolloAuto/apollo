load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package", "apollo_cc_library", "apollo_cc_test")

package(default_visibility = ["//visibility:public"])

apollo_cc_library(
    name = "shared_library",
    srcs = ["shared_library.cc"],
    hdrs = ["shared_library.h", "exceptions.h"],
    linkopts = ["-ldl"],
)

cc_binary(
    name = "libcyber_sample.so",
    testonly = True,
    linkshared = True,
    linkstatic = False,
    visibility = ["//visibility:private"],
    deps = [
        ":sample_lib",
    ],
)

cc_library(
    name = "sample_lib",
    testonly = True,
    srcs = ["sample.cc"],
    hdrs = ["sample.h"],
    linkopts = ["-lm"],
    visibility = ["//visibility:private"],
)

cc_test(
    name = "shared_library_test",
    size = "small",
    srcs = ["shared_library_test.cc"],
    data = [
        ":libcyber_sample.so",
    ],
    deps = [
        ":shared_library",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_package()
cpplint()
