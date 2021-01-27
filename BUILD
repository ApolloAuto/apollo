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
        "//modules/dreamview:install",
        "//modules/drivers:install",
        "//modules/monitor:install",
        "//modules/perception:install",
        "//modules/planning:install",
        "//modules/prediction:install",
        "//scripts:install",
    ],
)
