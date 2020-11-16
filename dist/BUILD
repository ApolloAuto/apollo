load("@rules_pkg//:pkg.bzl", "pkg_tar")

package(
    default_visibility = ["//visibility:public"],
)

pkg_tar(
    name = "apollo-dist",
    deps = [
        ":cyber-dist",
    ],
)

pkg_tar(
    name = "cyber-dist",
    deps = [
        ":cyber",
        ":mainboard",
    ],
)

pkg_tar(
    name = "cyber",
    srcs = [
        "//cyber",
    ],
    mode = "0644",
    package_dir = "lib",
    strip_prefix = "/cyber",
)

pkg_tar(
    name = "mainboard",
    srcs = [
        "//cyber:mainboard",
    ],
    mode = "0755",
    package_dir = "bin",
    strip_prefix = "/cyber",
)
