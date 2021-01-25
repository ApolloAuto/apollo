load("//tools/install:install.bzl", "install")

package(
    default_visibility = ["//visibility:public"],
)

exports_files([
    "CPPLINT.cfg",
    "tox.ini",
])

install(
    name = "install",
    deps = [
        "//cyber:install",
        "//cyber/examples:install",
        "//scripts:install",
        # "//modules/control:install",
        # "//modules/drivers:install",
    ],
)
