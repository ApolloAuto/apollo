load("//tools/install:install.bzl", "install")

package(
    default_visibility = ["//visibility:public"],
)

install(
    name = "install_scripts",
    data = [
        ":apollo_base",
    ],
    data_dest = "scripts/src",
)

install(
    name = "install",
    data = [":cyberfile.xml"],
    data_dest = "scripts",
    deps = ["install_scripts"],
)


filegroup(
    name = "apollo_base",
    srcs = glob(["*"]),
)
