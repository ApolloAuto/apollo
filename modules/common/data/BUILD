load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "vehicle_config_data",
    srcs = ["vehicle_param.pb.txt"],
)

filegroup(
    name = "global_flagfile",
    srcs = ["global_flagfile.txt"],
)

install(
    name = "install",
    data = [
        ":global_flagfile",
        ":vehicle_config_data",
    ],
)
