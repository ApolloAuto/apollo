load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

CANBUS_COPTS = ["-DMODULE_NAME=\\\"canbus\\\""]


apollo_component(
    name = "libserial_component.so",
    srcs = ["serial_component.cc"],
    hdrs = [
        "serial_component.h",
        "vehicle_factory.h",
        "base_control.h",
    ],
    copts = CANBUS_COPTS,
    deps = [
        "//modules/serial/vehicle/ros:ros_control",
        "//cyber",
        "@com_github_gflags_gflags//:gflags",
    ],
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/**",
        "dag/*.dag",
        "launch/*.launch",
    ]),
)

apollo_package()
cpplint()
