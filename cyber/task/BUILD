load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_package", "apollo_cc_test")

package(default_visibility = ["//visibility:public"])

apollo_cc_library(
    name = "cyber_task",
    hdrs = ["task.h", "task_manager.h"],
    srcs = ["task_manager.cc"],
    copts = ["-faligned-new"],
    deps = [
        "//cyber/scheduler:cyber_scheduler",
    ],
)

apollo_cc_test(
    name = "task_test",
    size = "small",
    srcs = ["task_test.cc"],
    deps = [
        "//cyber",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_package()
cpplint()
