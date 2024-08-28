load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_component", "apollo_package")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*.txt",
        "conf/*.conf",
        "dag/*.dag",
        "launch/*.launch",
        "params/*.yaml",
    ]),
)

apollo_package()

cpplint()
