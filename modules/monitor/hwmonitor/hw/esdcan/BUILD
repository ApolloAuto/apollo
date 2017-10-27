load("//tools:cpplint.bzl", "cpplint")

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
        "//modules/common:log",
        "//modules/common:macro",
        "//modules/common/util:string_util",
        "//modules/monitor/common:hw_checker",
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
        "//modules/common:log",
        "//modules/common:macro",
        "//modules/common/util:string_util",
        "//modules/monitor/common:hw_checker",
        "//third_party/can_card_library/esd_can",
    ],
)

cpplint()
