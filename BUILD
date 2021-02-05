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
        "//docker/scripts:install",
        "//modules/audio:install",
        "//modules/canbus:install",
        "//modules/control:install",
        "//modules/dreamview:install",
        "//modules/drivers/camera:install",
        "//modules/drivers/gnss:install",
        "//modules/guardian:install",
        "//modules/monitor:install",
        "//modules/perception:install",
        "//modules/planning:install",
        "//modules/prediction:install",
        "//modules/storytelling:install",
        "//modules/transform:install",
        "//scripts:install",
    ],
)
