load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
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
        "udp_input.cc",
        "hesai40_parser.cc",
        "hesai64_parser.cc",
        "parser.cc",
        "tcp_cmd_client.cc",
    ],
    hdrs = [
        "const_var.h",
        "driver.h",
        "hesai_driver_component.h",
        "hesai_convert_component.h",
        "udp_input.h",
        "parser.h",
        "tcp_cmd_client.h",
        "type_defs.h",
    ],
    copts = ['-DMODULE_NAME=\\"hesai\\"'],
    deps = [
        "//cyber",
        "//modules/common/util",
        "//modules/drivers/hesai/proto:hesai_proto",
        "//modules/drivers/proto:sensor_proto",
    ],
)

cpplint()
