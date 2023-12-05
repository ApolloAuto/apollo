load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_component", "apollo_package")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

# install(
#     name = "install",
#     data = [
#         ":runtime_data",
#     ],
#     data_dest = "drivers/addition_data/lidar/lslidar",
#     library_dest = "drivers/lib/lidar/lslidar",
#     targets = [
#         "//modules/drivers/lidar/lslidar/driver:liblslidar_driver_component.so",
#         "//modules/drivers/lidar/lslidar/parser:liblslidar_convert_component.so",
#     ],
# )

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

apollo_package()

cpplint()
