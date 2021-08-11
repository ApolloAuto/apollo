load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    data = [
        ":calibrated_vehicles",
    ],
)

filegroup(
    name = "calibrated_vehicles",
    srcs = glob([
        "data/**",
    ]),
)
