package(default_visibility = ["//visibility:public"])

cc_library(
    name = "esdcan_checker_lib",
    srcs = [
        "esdcan_checker.cc",
        "esdcan_err_str.cc",
        "esdcan_test.cc",
        "esdcan_utils.cc",
    ],
    hdrs = glob(["*.h"]),
    deps = [
        "//modules/monitor/common:interface",
        "//modules/monitor/common:platform_common_lib",
        "//modules/monitor/hwmonitor/hw:platform_hw_log_module_lib",
        "//third_party/can_card_library/esd_can",
    ],
)

cc_library(
    name = "mock_esdcan_checker_lib",
    srcs = [
        "esdcan_checker.cc",
        "esdcan_err_str.cc",
        "esdcan_test_mock.cc",
        "esdcan_utils.cc",
    ],
    hdrs = glob(["*.h"]),
    deps = [
        "//modules/monitor/common:interface",
        "//modules/monitor/common:platform_common_lib",
        "//modules/monitor/hwmonitor/hw:platform_hw_log_module_lib",
        "//third_party/can_card_library/esd_can",
    ],
)
