# JSON for Modern C++
package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "tf2",
    srcs = [
        "src/buffer_core.cpp",
        "src/cache.cpp",
        "src/static_cache.cpp",
        "src/time.cpp",
    ],
    hdrs = glob(["include/**/*.h"]),
    includes = ["include"],
    deps = [
        "//third_party:boost",
    ],
)
