load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package", "apollo_cc_library", "apollo_cc_test", "apollo_cc_binary")

package(default_visibility = ["//visibility:public"])

apollo_cc_library(
    name = "shared_library",
    srcs = ["shared_library.cc"],
    hdrs = ["shared_library.h", "exceptions.h"],
    linkopts = ["-ldl"],
)

apollo_cc_binary(
    name = "libcyber_sample.so",
    srcs = ["sample.cc", "sample.h"],
    linkopts = ["-lm"],
    testonly = True,
    linkshared = True,
    linkstatic = False,
    visibility = ["//visibility:private"],
)

apollo_cc_test(
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
