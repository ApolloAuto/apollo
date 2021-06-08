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
        "//docs:install",
        "//modules/audio:install",
        "//modules/bridge:install",
        "//modules/canbus:install",
        "//modules/common:install",
        "//modules/contrib/cyber_bridge:install",
        "//modules/control:install",
        "//modules/dreamview:install",
        "//modules/drivers:install",
        "//modules/guardian:install",
        "//modules/localization:install",
        "//modules/map:install",
        "//modules/monitor:install",
        "//modules/perception:install",
        "//modules/planning:install",
        "//modules/prediction:install",
        "//modules/routing:install",
        "//modules/storytelling:install",
        "//modules/task_manager:install",
        "//modules/third_party_perception:install",
        "//modules/tools:install",
        "//modules/transform:install",
        "//modules/v2x:install",
        "//scripts:install",
    ],
)
