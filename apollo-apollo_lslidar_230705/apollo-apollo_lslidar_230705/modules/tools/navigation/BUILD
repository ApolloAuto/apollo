load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "runtime_files",
    srcs = [
        "navigation_server_key",
    ],
)

install(
    name = "install",
    deps = [
        "//modules/tools/navigation/config:install",
        "//modules/tools/navigation/driving_behavior:install"
    ]
)