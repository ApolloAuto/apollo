load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "libsurestar_driver_component.so",
    linkopts = ["-shared"],
    linkstatic = False,
    deps = [":driver"],
)

cc_library(
    name = "driver",
    srcs =  [
        "src/component/component.cpp",
        "src/driver/driver.cpp",
        "src/driver/driver16.cpp",
        "src/lib/calibration.cpp",
        "src/lib/input.cpp",
        "src/lib/pcap_input.cpp",
        "src/lib/socket_input.cpp",
        "src/parser/convert.cpp",
        "src/parser/surestar_parser.cpp",
        "src/parser/surestar_status.cpp",
        "src/parser/surestar16_parser.cpp",
    ],
    hdrs = [
        "include/component/component_convert.h",
        "include/component/component_driver.h",
        "include/driver/driver.h",
        "include/lib/calibration.h",
        "include/lib/const_variables.h",
        "include/lib/data_type.h",
        "include/lib/input.h",
        "include/lib/pcap_input.h",
        "include/lib/socket_input.h",
        "include/parser/convert.h",
        "include/parser/surestar_parser.h",
        "include/parser/surestar_status.h",
    ],
    copts = ['-DMODULE_NAME=\\"surestar\\"'],
    deps = [
        "//cyber",
        "//modules/common/util",
        "//modules/drivers/lidar_surestar/proto:surestar_proto",
        "//modules/drivers/proto:sensor_proto",
        "@pcl",
        "@eigen",
        "@yaml_cpp//:yaml",
    ],
)

cpplint()
