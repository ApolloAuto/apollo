load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "perception_testdata",
    srcs = glob([
        "testdata/**",
    ]),
)

install(
    name = "install",
    deps = [
        "//modules/perception/onboard/component:install",
    ],
)

cpplint()
