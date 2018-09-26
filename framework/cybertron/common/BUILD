load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "common_files",
    srcs = glob(
        [
            "**/*.h",
            "**/*.cc",
            "**/*.cpp",
            "**/*.hpp",
        ],
    ),
)

cpplint()
