load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "cybertron",
    srcs = glob([
        "install/lib/libcybertron*.so",
        "install/third_party/**/libfast*.so",
    ]),
    hdrs = glob([
        "install/include/**/*.h",
        "third_party/**/*.h",
    ]),
    includes = [
        "install/include",
        "third_party/fast-rtps/include",
    ],
    deps = [
        ":cybertron_common",
        "//external:gflags",
        "@glog",
    ],
)

cc_library(
    name = "cybertron_common",
    srcs = glob([
        "install/lib/libcybertron_common.so",
    ]),
    hdrs = glob([
        "install/include/cybertron/common/*.h",
    ]),
    includes = [
        "install/include",
    ],
    deps = [
        ":cybertron_base",
        "@glog",
    ],
)

cc_library(
    name = "cybertron_base",
    hdrs = glob([
        "install/include/cybertron/base/*.h",
    ]),
    includes = [
        "install/include",
    ],
)

cpplint()
