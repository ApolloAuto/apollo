load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "canbus_neolix_edu_protocol",
    srcs = glob(
        ["*.cc"],
        exclude = ["*_test.cc"],
    ),
    hdrs = glob([
        "*.h",
    ]),
    deps = [
        "//modules/canbus/proto:chassis_detail_cc_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "canbus_neolix_edu_protocol_test",
    size = "small",
    srcs = glob(["*_test.cc"]),
    deps = [
        "//modules/canbus/vehicle/neolix_edu/protocol:canbus_neolix_edu_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
