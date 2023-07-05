load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

HESAI_COPTS = ['-DMODULE_NAME=\\"hesai\\"']

install(
    name = "install",
    data_dest = "drivers/addition_data/lidar/hesai",
    library_dest = "drivers/lib/lidar/hesai",
    data = [
        ":runtime_data",
    ],
    targets = [
        "libhesai_driver_component.so",
    ],
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*.txt",
        "conf/*.conf",
        "dag/*.dag",
        "launch/*.launch",
    ]),
)

cc_binary(
    name = "libhesai_driver_component.so",
    linkshared = True,
    linkstatic = True,
    deps = [
        ":hesai_convert_component",
        "//modules/drivers/lidar/hesai/driver:hesai_component",
    ],
)

cc_library(
    name = "hesai_convert_component",
    srcs = ["parser/hesai_convert_component.cc"],
    hdrs = ["parser/hesai_convert_component.h"],
    copts = HESAI_COPTS,
    deps = [
        ":parser_factory",
        "//cyber",
        "//modules/drivers/lidar/hesai/driver",
        "//modules/drivers/lidar/proto:hesai_config_cc_proto",
    ],
    alwayslink = True,
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
        "//modules/drivers/lidar/proto:hesai_cc_proto",
        "//modules/drivers/lidar/proto:hesai_config_cc_proto",
        "//modules/common_msgs/sensor_msgs:pointcloud_cc_proto",
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
