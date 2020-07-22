load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "canbus_neolix_edu_protocol",
    srcs = glob(["*.cc",],exclude = ["*_test.cc"],),
    hdrs = glob([
        "*.h",
    ]),
    deps = [
        "//modules/drivers/canbus/common:canbus_common",
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
    ],
)
cc_test(
    name = "canbus_neolix_edu_protocol_test",
    size = "small",
    srcs = glob(["*_test.cc",],),
    deps = [
        "//modules/canbus/vehicle/neolix_edu/protocol:canbus_neolix_edu_protocol",
        "@gtest//:main",
    ],
)

cpplint()
