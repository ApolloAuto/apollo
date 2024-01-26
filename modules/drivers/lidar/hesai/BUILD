load("//tools:apollo_package.bzl", "apollo_package", "apollo_cc_library", "apollo_component")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

HESAI_COPTS = ['-DMODULE_NAME=\\"hesai\\"']

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*.txt",
        "conf/*.conf",
        "dag/*.dag",
        "launch/*.launch",
    ]),
)

apollo_component(
    name = "libhesai_driver_component.so",
    srcs = ["parser/hesai_convert_component.cc",],
    hdrs = ["parser/hesai_convert_component.h",],
    deps = [
        ":hesai_convert_component",
        "//modules/drivers/lidar/hesai/driver:hesai_component",
    ],
)

apollo_cc_library(
    name = "hesai_convert_component",
    srcs = [
        "parser/parser_factory.cc",
        "parser/hesai64_parser.cc",
        "parser/hesai40_parser.cc",
        "parser/parser.cc",
        "input/udp_input.cc",
        "parser/tcp_cmd_client.cc",
    ],
    hdrs = [
        "parser/parser_factory.h",
        "parser/hesai64_parser.h",
        "parser/hesai40_parser.h",
        "parser/parser.h",
        "input/udp_input.h",
        "parser/tcp_cmd_client.h",
        "common/type_defs.h",
        "common/const_var.h",
    ],
    copts = HESAI_COPTS,
    deps = [
        "//cyber",
        "//modules/drivers/lidar/proto:hesai_cc_proto",
        "//modules/drivers/lidar/proto:hesai_config_cc_proto",
        "//modules/common_msgs/sensor_msgs:pointcloud_cc_proto",
        "//modules/drivers/lidar/common/driver_factory:apollo_lidar_driver_base",
        "//modules/drivers/lidar/proto:config_cc_proto",
    ],
)

apollo_package()
cpplint()
