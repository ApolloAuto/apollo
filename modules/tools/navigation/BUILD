# load("//tools/install:install.bzl", "install")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "runtime_files",
    srcs = [
        "navigation_server_key",
    ],
)

# install(
#     name = "install",
#     deps = [
#         "//modules/tools/navigation/config:install",
#         "//modules/tools/navigation/driving_behavior:install"
#     ]
# )
apollo_package()