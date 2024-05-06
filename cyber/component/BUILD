load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package", "apollo_cc_library", "apollo_cc_test")

package(default_visibility = ["//visibility:public"])

apollo_cc_library(
    name = "cyber_component",
    hdrs = [
        "component.h",
        "timer_component.h",
        "component_base.h",
    ],
    srcs = [
        "timer_component.cc"
    ],
    deps = [
        "//cyber/scheduler:cyber_scheduler",
        "//cyber/blocker:cyber_blocker",
        "//cyber/timer:cyber_timer",
        "//cyber/transport:cyber_transport",
        "//cyber/base:cyber_base",
        "//cyber/class_loader:cyber_class_loader",
        "//cyber/node:cyber_node",
        "@com_github_gflags_gflags//:gflags",
    ],
)

apollo_cc_test(
    name = "component_test",
    size = "small",
    srcs = ["component_test.cc"],
    deps = [
        "//cyber",
        "@com_google_googletest//:gtest",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "timer_component_test",
    size = "small",
    srcs = ["timer_component_test.cc"],
    deps = [
        "//cyber",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_package()
cpplint()
