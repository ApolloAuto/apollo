load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "cyber_recorder",
    srcs = glob([
        "cyber_recorder/*.cpp",
        "cyber_recorder/*.h",
    ]),
    deps = [
        "//cybertron:cybertron_core",
    ],
    linkopts = [
        "-pthread",
    ],
    #linkstatic = False,
)

cc_binary(
    name = "cyber_monitor",
    srcs = glob([
        "cvt/monitor/*.cpp",
        "cvt/monitor/*.h",
    ]),
    deps = [
        "//cybertron:cybertron_core",
    ],
    linkopts = [
        "-pthread",
        "-lncurses"
    ],
    #linkstatic = False,
)

py_binary(
    name = "extrinsics_broadcaster",
    srcs = [
        "extrinsics_broadcaster/bin/extrinsics_broadcaster.py",
    ],
)

#cpplint()
