load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "drivers_conti_radar_protocol",
    srcs = glob([
        "*.cc",
    ]),
    hdrs = glob([
        "*.h",
    ]),
    deps = [
        "//modules/common/proto:header_cc_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/proto:can_card_parameter_cc_proto",
        "//modules/drivers/proto:conti_radar_cc_proto",
        "//modules/drivers/radar/conti_radar/proto:conti_radar_conf_cc_proto",
    ],
)

cpplint()
