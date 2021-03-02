load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "perception_testdata",
    srcs = glob([
        "testdata/**",
    ]),
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/**",
        "dag/*.dag",
        "data/**",
        "launch/*.launch",
        "models/**",
    ]),
)

install(
    name = "install",
    data = [
        ":runtime_data",
    ],
    deps = [
        "//modules/perception/submodules:install",
    ],
)

cpplint()
