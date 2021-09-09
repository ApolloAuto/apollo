load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

OUSTER_COPTS = ['-DMODULE_NAME=\\"ouster\\"']

install(
    name = "install",
    data = [
        ":runtime_data",
    ],
    targets = [
        "libouster_driver_component.so",
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
    name = "libouster_driver_component.so",
    linkshared = True,
    linkstatic = False,
    linkopts = [
    	"-shared",
        "-L/apollo/modules/drivers/lidar/ouster/lib",
        "-Wl,-rpath=/apollo/modules/drivers/lidar/ouster/lib",      
        "-louster_driver",
    ],
    copts = OUSTER_COPTS,
    deps = [
        "//cyber",
        "//modules/common/util",
        "//modules/drivers/lidar/proto:ouster_cc_proto",
        "//modules/drivers/lidar/proto:ouster_config_cc_proto",
        "//modules/drivers/proto:pointcloud_cc_proto",
    ],
)

cpplint()
