# load("//tools/install:install.bzl", "install")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

# install(
#     name = "install",
#     data_dest = "tools/map_datachecker",
#     data = [":scripts"],
#     visibility = ["//visibility:public"],
# )

filegroup(
    name = "scripts",
    srcs = glob([
        "*.sh",
    ]),
)

apollo_package()