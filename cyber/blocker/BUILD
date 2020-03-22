load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "blocker_manager",
    srcs = ["blocker_manager.cc"],
    hdrs = ["blocker_manager.h"],
    deps = [
        ":blocker",
    ],
)

cc_test(
    name = "blocker_manager_test",
    size = "small",
    srcs = ["blocker_manager_test.cc"],
    deps = [
        "//cyber",
        "//cyber/proto:unit_test_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "blocker",
    hdrs = ["blocker.h"],
)

cc_test(
    name = "blocker_test",
    size = "small",
    srcs = ["blocker_test.cc"],
    deps = [
        "//cyber",
        "//cyber/proto:unit_test_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "intra_reader",
    hdrs = ["intra_reader.h"],
    deps = [
        ":blocker_manager",
    ],
)

cc_library(
    name = "intra_writer",
    hdrs = ["intra_writer.h"],
    deps = [
        ":blocker_manager",
    ],
)

cpplint()
