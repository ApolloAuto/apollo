load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "libhesai_driver_component.so",
    linkopts = ["-shared"],
    linkstatic = False,
    deps = [":driver"],
)

cc_library(
    name = "driver",
    srcs = [
        "component.cc",
        "driver.cc",
        "hesai40_parser.cc",
        "hesai64_parser.cc",
        "parser.cc",
        "tcp_cmd_client.cc",
        "udp_input.cc",
    ],
    hdrs = [
        "const_var.h",
        "driver.h",
        "hesai_convert_component.h",
        "hesai_driver_component.h",
        "parser.h",
        "tcp_cmd_client.h",
        "type_defs.h",
        "udp_input.h",
    ],
    copts = ['-DMODULE_NAME=\\"hesai\\"'],
    deps = [
        "//cyber",
        "//modules/common/util",
        "//modules/drivers/hesai/proto:config_cc_proto",
        "//modules/drivers/hesai/proto:hesai_cc_proto",
        "//modules/drivers/proto:pointcloud_cc_proto",
    ],
)

cpplint()
