load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    data_dest = "drivers/addition_data/lidar/velodyne",
    library_dest = "drivers/lib/lidar/velodyne",
    data = [
        ":runtime_data",
    ],
    targets = [
        "//modules/drivers/lidar/velodyne/compensator:libvelodyne_compensator_component.so",
        "//modules/drivers/lidar/velodyne/driver:libvelodyne_driver_component.so",
        "//modules/drivers/lidar/velodyne/fusion:libvelodyne_fusion_component.so",
        "//modules/drivers/lidar/velodyne/parser:libvelodyne_convert_component.so",
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
