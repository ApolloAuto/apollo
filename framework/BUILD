package(default_visibility = ["//visibility:public"])

# https://docs.bazel.build/versions/master/be/c-cpp.html#cc_library
cc_library(
    name = "cybertron",
    srcs = glob([
        "install/lib/libcybertron*.so",
    ]),
    hdrs = glob([
        "install/include/**/*.h",
        "third_party/**/*.h"    
    ]),
    includes = [
        "install/include",
        "third_party/fast-rtps/include"
    ],
    deps = [
        ":cybertron_common",
        "@glog//:glog",
        "//external:gflags",
    ]
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
        "@glog//:glog"
    ]
)

cc_library(
    name = "cybertron_base",
    hdrs = glob([
        "install/include/cybertron/base/*.h", 
    ]),
    includes = [
        "install/include",
    ]
)
