load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

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
    srcs = ["hesai_convert_component.cc"],
    hdrs = ["hesai_convert_component.h"],
    copts = ['-DMODULE_NAME=\\"hesai\\"'],
    deps = [
        ":driver",
        ":parser_factory",
        "//cyber",
        "//modules/drivers/hesai/proto:config_cc_proto",
    ],
)

cc_library(
    name = "hesai_component",
    srcs = ["hesai_component.cc"],
    hdrs = ["hesai_component.h"],
    copts = ['-DMODULE_NAME=\\"hesai\\"'],
    deps = [
        ":driver",
        ":parser_factory",
        "//cyber",
        "//modules/drivers/hesai/proto:config_cc_proto",
    ],
)

cc_library(
    name = "driver",
    srcs = ["driver.cc"],
    hdrs = ["driver.h"],
    deps = [
        ":parser",
        ":type_defs",
        ":udp_input",
        "//cyber",
    ],
)

cc_library(
    name = "parser_factory",
    srcs = ["parser_factory.cc"],
    hdrs = ["parser_factory.h"],
    deps = [
        ":hesai40_parser",
        ":hesai64_parser",
        ":parser",
        "//cyber",
    ],
)

cc_library(
    name = "hesai64_parser",
    srcs = ["hesai64_parser.cc"],
    hdrs = ["hesai64_parser.h"],
    deps = [
        ":parser",
        "//cyber",
    ],
)

cc_library(
    name = "hesai40_parser",
    srcs = ["hesai40_parser.cc"],
    hdrs = ["hesai40_parser.h"],
    deps = [
        ":parser",
        "//cyber",
    ],
)

cc_library(
    name = "parser",
    srcs = ["parser.cc"],
    hdrs = ["parser.h"],
    deps = [
        ":tcp_cmd_client",
        ":type_defs",
        "//cyber",
        "//modules/drivers/hesai/proto:config_cc_proto",
        "//modules/drivers/hesai/proto:hesai_cc_proto",
        "//modules/drivers/proto:pointcloud_cc_proto",
    ],
)

cc_library(
    name = "udp_input",
    srcs = ["udp_input.cc"],
    hdrs = ["udp_input.h"],
    deps = [
        ":type_defs",
        "//cyber",
    ],
)

cc_library(
    name = "tcp_cmd_client",
    srcs = ["tcp_cmd_client.cc"],
    hdrs = ["tcp_cmd_client.h"],
    deps = ["//cyber"],
)

cc_library(
    name = "type_defs",
    hdrs = ["type_defs.h"],
    deps = [":const_var"],
)

cc_library(
    name = "const_var",
    hdrs = ["const_var.h"],
)

cpplint()
