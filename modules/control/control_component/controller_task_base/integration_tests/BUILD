load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package", "apollo_cc_library", "apollo_cc_test")

package(default_visibility = ["//visibility:public"])

apollo_cc_library(
    name = "control_test_base",
    srcs = ["control_test_base.cc"],
    hdrs = ["control_test_base.h"],
    copts = ["-fno-access-control"],
    deps = [
        "//cyber",
        "//modules/common_msgs/basic_msgs:geometry_cc_proto",
        # We strongly discourage depending on targets,
        # implicitly declared by the apollo_component rule,
        # which can lead to issues such as plugin loading failures.
        # In this case, since the dynamic library is only used by the single test,
        # it directly depends on the implicitly declared target "DO_NOT_IMPORT_*".
        "//modules/control/control_component:DO_NOT_IMPORT_control_component",
        "//modules/control/control_component/common:control_gflags",
        "//modules/control/control_component/controller_task_base:control_task_agent",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "simple_control_test",
    size = "small",
    srcs = ["simple_control_test.cc"],
    copts = ["-fno-access-control"],
    linkstatic = True,
    data = ["//modules/control/control_component:test_data"],
    deps = [
        ":control_test_base",
        "@com_google_googletest//:gtest",
    ],
)

apollo_cc_test(
    name = "relative_position_test",
    size = "small",
    srcs = ["relative_position_test.cc"],
    copts = ["-fno-access-control"],
    linkstatic = True,
    data = ["//modules/control/control_component:test_data"],
    deps = [
        ":control_test_base",
        "@com_google_googletest//:gtest",
    ],
)

apollo_package()
cpplint()
