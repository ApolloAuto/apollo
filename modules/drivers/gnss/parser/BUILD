load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "gnss_parser",
    deps = [
        ":data_parser",
        ":novatel_parser",
        ":rtcm_parser",
        ":rtcm3_parser"
    ],
)

cc_library(
    name = "data_parser",
    srcs = ["data_parser.cc"],
    hdrs = [
        "parser.h",
        "data_parser.h"
        ],
    deps = [
        ":novatel_parser",
        "//external:gflags",
        "//modules/common:log",
        "//modules/common/monitor_log",
        "//modules/drivers/gnss/proto:gnss_proto",
        "//modules/drivers/gnss/util:gnss_util",
        "//modules/drivers/gnss:gnss_gflags",
        "//modules/localization/proto:imu_proto",
        "//modules/localization/proto:gps_proto",
        "//modules/common/adapters:adapter_manager",
        "@ros//:ros_common",
        "@proj4//:proj4"
    ],
)

cc_library(
    name = "novatel_parser",
    srcs = ["novatel_parser.cc"],
    hdrs = [
        "parser.h",
        "novatel_messages.h",
        "rtcm_decode.h"
        ],
    deps = [
        "//external:gflags",
        "//modules/common:log",
        "//modules/common/monitor_log",
        "//modules/drivers/gnss/proto:gnss_proto",
        "//modules/drivers/gnss/third_party:rtcm",
        "//modules/drivers/gnss/util:gnss_util",
        "@ros//:ros_common"
    ],
)

cc_library(
    name = "rtcm_parser",
    srcs = ["rtcm_parser.cc"],
    hdrs = [
        "parser.h",
        "rtcm_parser.h"
        ],
    deps = [
        "//external:gflags",
        "//modules/common:log",
        "//modules/common/monitor_log",
        "//modules/drivers/gnss/proto:gnss_proto",
        "//modules/drivers/gnss/util:gnss_util",
        "//modules/drivers/gnss:gnss_gflags",
        "//modules/common/adapters:adapter_manager",
        "@ros//:ros_common"
    ],
)

cc_library(
    name = "rtcm3_parser",
    srcs = ["rtcm3_parser.cc"],
    hdrs = [
        "parser.h",
        "rtcm_parser.h",
        "rtcm_decode.h"
        ],
    deps = [
        "//external:gflags",
        "//modules/common:log",
        "//modules/common/monitor_log",
        "//modules/drivers/gnss/proto:gnss_proto",
        "//modules/drivers/gnss/third_party:rtcm",
        "//modules/drivers/gnss/util:gnss_util",
        "@ros//:ros_common"
    ],
)

cpplint()
