load("//tools/install:install.bzl", "install")

package(
    default_visibility = ["//visibility:public"],
)

install(
    name = "install",
    data = [
        ":record_helper",
    ],
)

filegroup(
    name = "record_helper",
    srcs = [
        "record_helper.py",
    ],
)
