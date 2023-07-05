load("//tools/install:install.bzl", "install")

package(
    default_visibility = ["//visibility:public"],
)

install(
    name = "install",
    data = [
        ":run_files",
    ],
)

filegroup(
    name = "run_files",
    srcs = glob([
        "demo_guide/*.py",
    ]),
)
