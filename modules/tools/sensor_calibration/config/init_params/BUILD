load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "runtime_files",
    srcs = glob([
        "*",
    ]),
)

install(
    name = "install",
    data = [":runtime_files"],
    data_dest = "tools/sensor_calibration/config/init_parms"
)