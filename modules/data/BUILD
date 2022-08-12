load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    data = [
        ":runtime_data",
    ],
    deps = [
        "//modules/data/tools/smart_recorder:smart_recorder_binary",
    ],
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "tools/smart_recorder/conf/*.conf",
    ]),
)
