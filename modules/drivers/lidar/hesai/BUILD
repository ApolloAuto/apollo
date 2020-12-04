load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

HESAI_COPTS = ['-DMODULE_NAME=\\"hesai\\"']

cc_binary(
    name = "libhesai_driver_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [
        ":hesai_component",
        ":hesai_convert_component",
    ],
)

cc_library(
    name = "hesai_convert_component",
    srcs = ["parser/hesai_convert_component.cc"],
    hdrs = ["parser/hesai_convert_component.h"],
    copts = HESAI_COPTS,
    deps = [
        ":driver",
        ":parser_factory",
        "//cyber",
        "//modules/drivers/lidar/hesai/proto:config_cc_proto",
    ],
)

cc_library(
    name = "hesai_component",
    srcs = ["driver/hesai_component.cc"],
    hdrs = ["driver/hesai_component.h"],
    copts = HESAI_COPTS,
    deps = [
        ":driver",
        ":parser_factory",
        "//cyber",
        "//modules/drivers/lidar/hesai/proto:config_cc_proto",
    ],
)

cc_library(
    name = "driver",
    srcs = ["driver/driver.cc"],
    hdrs = ["driver/driver.h"],
    copts = HESAI_COPTS,
    deps = [
        ":parser",
        ":type_defs",
        ":udp_input",
        "//cyber",
    ],
)

cc_library(
    name = "parser_factory",
    srcs = ["parser/parser_factory.cc"],
    hdrs = ["parser/parser_factory.h"],
    copts = HESAI_COPTS,
    deps = [
        ":hesai40_parser",
        ":hesai64_parser",
        ":parser",
        "//cyber",
    ],
)

cc_library(
    name = "hesai64_parser",
    srcs = ["parser/hesai64_parser.cc"],
    hdrs = ["parser/hesai64_parser.h"],
    copts = HESAI_COPTS,
    deps = [
        ":parser",
        "//cyber",
    ],
)

cc_library(
    name = "hesai40_parser",
    srcs = ["parser/hesai40_parser.cc"],
    hdrs = ["parser/hesai40_parser.h"],
    copts = HESAI_COPTS,
    deps = [
        ":parser",
        "//cyber",
    ],
)

cc_library(
    name = "parser",
    srcs = ["parser/parser.cc"],
    hdrs = ["parser/parser.h"],
    copts = HESAI_COPTS,
    deps = [
        ":tcp_cmd_client",
        ":type_defs",
        "//cyber",
        "//modules/drivers/lidar/hesai/proto:config_cc_proto",
        "//modules/drivers/lidar/hesai/proto:hesai_cc_proto",
        "//modules/drivers/proto:pointcloud_cc_proto",
    ],
)

cc_library(
    name = "udp_input",
    srcs = ["input/udp_input.cc"],
    hdrs = ["input/udp_input.h"],
    copts = HESAI_COPTS,
    deps = [
        ":type_defs",
        "//cyber",
    ],
)

cc_library(
    name = "tcp_cmd_client",
    srcs = ["parser/tcp_cmd_client.cc"],
    hdrs = ["parser/tcp_cmd_client.h"],
    copts = HESAI_COPTS,
    deps = ["//cyber"],
)

cc_library(
    name = "type_defs",
    hdrs = ["common/type_defs.h"],
    deps = [":const_var"],
)

cc_library(
    name = "const_var",
    hdrs = ["common/const_var.h"],
)

cpplint()
