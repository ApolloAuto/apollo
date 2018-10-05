load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "cybertron",
    deps = [
        "//framework/cybertron:cybertron_core",
    ],
    includes = [
        ".",
    ],
    linkstatic = False,
)

cpplint()
