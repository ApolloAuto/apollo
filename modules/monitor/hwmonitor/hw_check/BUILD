load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "can_check",
    srcs = [
        "can_check.cc",
        "hw_chk_utils.h",
    ],
    deps = [
        "//external:gflags",
        "//modules/hmi/utils:hmi_status_helper",
        "//modules/monitor/hwmonitor/hw:platform_hw_log_module_lib",
        "//modules/monitor/hwmonitor/hw/esdcan:esdcan_checker_lib",
    ],
)

cc_binary(
    name = "gps_check",
    srcs = [
        "gps_check.cc",
        "hw_chk_utils.h",
    ],
    deps = [
        "//external:gflags",
        "//modules/common/proto:gnss_status_proto",
        "//modules/hmi/utils:hmi_status_helper",
        "//modules/monitor/hwmonitor/hw:platform_hw_log_module_lib",
        "//modules/monitor/hwmonitor/hw/esdcan:esdcan_checker_lib",
        "//third_party/ros:ros_common",
    ],
)

cc_binary(
    name = "mock_can_check",
    srcs = [
        "can_check.cc",
        "hw_chk_utils.h",
    ],
    defines = ["DEBUG"],
    deps = [
        "//external:gflags",
        "//modules/hmi/utils:hmi_status_helper",
        "//modules/monitor/hwmonitor/hw:platform_hw_log_module_lib",
        "//modules/monitor/hwmonitor/hw/esdcan:mock_esdcan_checker_lib",
    ],
)

cpplint()
