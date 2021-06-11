load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    data = [
        ":runtime_data",
    ],
    deps = [
        "//modules/v2x/fusion/apps:install",
    ],
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*.conf",
        "dag/*.dag",
        "data/**",
        "launch/*.launch",
        "fusion/test_data/**",
    ]),
)
