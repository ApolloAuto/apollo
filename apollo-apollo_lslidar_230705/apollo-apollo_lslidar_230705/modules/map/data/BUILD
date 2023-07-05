load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    data = [
        ":demo",
        ":borregas_ave",
    ],
)

filegroup(
    name = "map_sunnyvale_big_loop",
    testonly = True,
    srcs = glob([
        "sunnyvale_big_loop/**",
    ]),
)

filegroup(
    name = "map_sunnyvale_loop",
    testonly = True,
    srcs = glob([
        "sunnyvale_loop/**",
    ]),
)

filegroup(
    name = "demo",
    srcs = glob([
        "demo/**",
    ])
)

filegroup(
    name = "borregas_ave",
    srcs = glob([
        "borregas_ave/**",
    ])
)
