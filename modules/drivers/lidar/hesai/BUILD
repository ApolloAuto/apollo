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
        "driver/component.cc",
        "driver/driver.cc",
        "input/udp_input.cc",
        "parser/hesai40_parser.cc",
        "parser/hesai64_parser.cc",
        "parser/parser.cc",
        "parser/tcp_cmd_client.cc",
    ],
    hdrs = [
        "common/const_var.h",
        "driver/driver.h",
        "driver/hesai_driver_component.h",
        "driver/hesai_convert_component.h",
        "input/udp_input.h",
        "parser/parser.h",
        "parser/tcp_cmd_client.h",
        "common/type_defs.h",
    ],
    copts = ['-DMODULE_NAME=\\"hesai\\"'],
    deps = [
        "//cyber",
        "//modules/common/util",
        "//modules/drivers/lidar/hesai/proto:hesai_proto",
        "//modules/drivers/proto:sensor_proto",
    ],
)

cpplint()
