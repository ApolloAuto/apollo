load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_inc_library(
    name = "hw_chk_utils",
    hdrs = [
        "hw_chk_utils.h",
    ],
    deps = [
        "//modules/monitor/common:hw_checker",
    ],
)

cc_binary(
    name = "can_check",
    srcs = [
        "can_check.cc",
    ],
    deps = [
        ":hw_chk_utils",
        "//external:gflags",
        "//modules/hmi/utils:hmi_status_helper",
        "//modules/monitor/hwmonitor/hw:hw_log_module",
        "//modules/monitor/hwmonitor/hw/esdcan:esdcan_checker_lib",
        "//modules/monitor/common:can_checker_factory",
        "//modules/canbus/proto:canbus_proto",
        "//modules/canbus/common:canbus_common",
        "//modules/common/util:util",
    ],
)

cc_binary(
    name = "gps_check",
    srcs = [
        "gps_check.cc",
    ],
    deps = [
        ":hw_chk_utils",
        "//external:gflags",
        "//modules/common/proto:gnss_status_proto",
        "//modules/hmi/utils:hmi_status_helper",
        "//modules/monitor/hwmonitor/hw:hw_log_module",
        "@ros//:ros_common",
    ],
)

cc_binary(
    name = "mock_can_check",
    srcs = [
        "can_check.cc",
    ],
    defines = ["DEBUG"],
    deps = [
        ":hw_chk_utils",
        "//external:gflags",
        "//modules/hmi/utils:hmi_status_helper",
        "//modules/monitor/hwmonitor/hw:hw_log_module",
        "//modules/monitor/hwmonitor/hw/esdcan:mock_esdcan_checker_lib",
        "//modules/monitor/common:can_checker_factory",
        "//modules/canbus/proto:canbus_proto",
        "//modules/canbus/common:canbus_common",
        "//modules/common/util:util",
    ],
)

cpplint()
