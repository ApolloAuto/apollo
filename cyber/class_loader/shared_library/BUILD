load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "cyber_class_loader_shared_library_hdrs",
    srcs = glob([
        "*.h",
    ]),
)
cc_library(
    name = "shared_library",
    srcs = ["shared_library.cc"],
    hdrs = ["shared_library.h"],
    linkopts = ["-ldl"],
    deps = [
        ":exceptions",
    ],
)

cc_library(
    name = "exceptions",
    hdrs = ["exceptions.h"],
)

cc_binary(
    name = "libsample.so",
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
        ":libsample.so",
    ],
    deps = [
        ":shared_library",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
