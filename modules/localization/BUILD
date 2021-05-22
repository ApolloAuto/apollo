load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "localization_testdata",
    srcs = glob(["testdata/*"]),
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*.conf",
        "conf/*.pb.txt",
        "dag/*.dag",
        "launch/*.launch",
    ]),
)

install(
    name = "install",
    data = [
        ":runtime_data",
    ],
    deps = [
        "//modules/localization/msf:install",
        "//modules/localization/rtk:install",
        "//modules/localization/ndt:install",
    ],
)

cpplint()
