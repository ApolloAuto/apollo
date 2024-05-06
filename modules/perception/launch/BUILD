load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "launch_files",
    srcs = glob([
        "*.launch",
    ])
)

apollo_package()
cpplint()
