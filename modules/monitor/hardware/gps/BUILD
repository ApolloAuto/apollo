load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "gps_check",
    srcs = [
        "gps_check.cc",
    ],
    deps = [
        "//external:gflags",
        "//modules/common:log",
        "//modules/common/proto:gnss_status_proto",
        "//modules/hmi/utils:hmi_status_helper",
        "//modules/monitor/hardware:hardware_check_utils",
        "@ros//:ros_common",
    ],
)

cpplint()
