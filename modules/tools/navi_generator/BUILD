load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "frontend",
    srcs = glob(["frontend/html/**/*"]),
)

cc_binary(
    name = "navi_generator",
    data = [
        ":frontend",
    ],
    deps = [
        "//modules/tools/navi_generator/backend",
    ],
)

cpplint()