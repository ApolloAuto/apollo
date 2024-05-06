load("//tools:apollo_package.bzl", "apollo_package", "apollo_component", "apollo_cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

apollo_cc_library(
    name = "drivers_racobit_radar_protocol",
    srcs = glob([
        "*.cc",
    ]),
    hdrs = glob([
        "*.h",
    ]),
    deps = [
        "//modules/common_msgs/basic_msgs:header_cc_proto",
        "//modules/drivers/canbus:apollo_drivers_canbus",
        "//modules/common_msgs/drivers_msgs:can_card_parameter_cc_proto",
        "//modules/common_msgs/sensor_msgs:racobit_radar_cc_proto",
        "//modules/drivers/radar/racobit_radar/proto:racobit_radar_conf_cc_proto",
    ],
)

apollo_package()
cpplint()
