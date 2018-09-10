package(default_visibility = ["//visibility:public"])

# https://docs.bazel.build/versions/master/be/c-cpp.html#cc_library
cc_library(
    name = "cybertron",
    srcs = glob([
        "install/lib/libcybertron*.so",
        "install/lib/libtf2.so",
        "install/third_party/libfast*.so",
        "install/third_party/libglog.so"
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
        "@glog//:glog"
    ]
)
