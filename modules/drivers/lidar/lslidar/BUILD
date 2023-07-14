load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    data_dest = "drivers/addition_data/lidar/lslidar",
    library_dest = "drivers/lib/lidar/lslidar",
    data = [
        ":runtime_data",
    ],
    targets = [
        "//modules/drivers/lidar/lslidar/driver:liblslidar_driver_component.so",
        "//modules/drivers/lidar/lslidar/parser:liblslidar_convert_component.so",
    ],
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*.txt",
        "conf/*.conf",
        "dag/*.dag",
        "launch/*.launch",
        "params/*.yaml",
    ]),
)
