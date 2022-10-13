load("//tools/install:install.bzl", "install")
package(default_visibility = ["//visibility:public"])

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/**",
        "dag/*.dag",
        "data/**",
        "launch/*.launch",
    ]),
)

install(
    name = "install",
    data_dest = "perception/addition_data/production",
    data = [
        ":runtime_data",
    ],
)
