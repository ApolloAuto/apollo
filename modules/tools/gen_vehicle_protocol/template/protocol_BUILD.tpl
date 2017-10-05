load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "canbus_%(car_type)s_protocol",
    srcs = glob([
        "*.cc",
    ]),
    hdrs = glob([
        "*.h",
    ]),
    deps = [
        "//modules/canbus/common:canbus_common",
        "//modules/canbus/proto:canbus_proto",
        "//modules/canbus/vehicle:message_manager_base",
    ],
)

cpplint()