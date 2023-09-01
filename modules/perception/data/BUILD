load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "data_files",
    srcs = glob([
        "flag/**",
        "params/**",
        "conf/**",
    ])
)

apollo_package()
cpplint()
