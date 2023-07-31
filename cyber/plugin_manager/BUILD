load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
# load("//tools/install:install.bzl", "install")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

# install(
#     name = "install",
#     library_dest = "cyber/lib",
#     targets = [
#         ":libplugin_manager.so",
#     ],
# )

cc_binary(
    name = "libplugin_manager.so",
    srcs = [
        "plugin_description.h",
        "plugin_manager.cc",
        "plugin_manager.h",
    ],
    linkshared = True,
    linkstatic = True,
    deps = [
        "//cyber/class_loader:class_loader_manager",
        "//cyber/common",
        "@tinyxml2",
    ],
)

cc_library(
    name = "plugin_manager",
    srcs = [
        ":libplugin_manager.so",
    ],
    hdrs = [
        "plugin_description.h",
        "plugin_manager.h",
    ],
    alwayslink = True,
)

apollo_package()
cpplint()
