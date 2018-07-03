load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

# Run build_and_install.sh to install Pandora driver. Here we just include the
# files into lint check.
filegroup(
    name = "pandora",
    srcs = glob([
        "**/*.cc",
        "**/*.h",
    ]),
)

cpplint()
