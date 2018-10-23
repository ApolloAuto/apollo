load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "cyber_monitor",
    srcs = glob([
        "cvt/monitor/*.cc",
        "cvt/monitor/*.h",
    ]),
    deps = [
        "//cyber:cyber_core",
    ],
    linkopts = [
        "-pthread",
        "-lncurses"
    ],
    #linkstatic = False,
)

#cpplint()
