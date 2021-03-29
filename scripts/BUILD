load("//tools/install:install.bzl", "install")

package(
    default_visibility = ["//visibility:public"],
)

install(
    name = "install",
    data = [
        ":apollo_base",
        ":bootstrap",
        ":bridge",
        ":docker_utils",
    ],
)

filegroup(
    name = "bootstrap",
    srcs = [
        "bootstrap.sh",
        "bootstrap_lgsvl.sh",
        "dreamview.sh",
        "monitor.sh",
    ],
)

filegroup(
    name = "bridge",
    srcs = [
        "bridge.sh",
    ],
)

filegroup(
    name = "docker_utils",
    srcs = [
        ":docker_start_user.sh",
    ],
)

filegroup(
    name = "apollo_base",
    srcs = [
        "apollo.bashrc",
        "apollo_base.sh",
        "common.bashrc",
    ],
)
