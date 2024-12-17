load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "protobuf_lite",
    srcs = [
        "lib64/libprotobuf-lite.so",
    ],
    hdrs = glob(["include/google/protobuf/**/*.h"]),
    linkopts = [
        "-lprotobuf_lite",
    ],
)

cc_library(
    name = "protobuf",
    srcs = [
        "lib64/libprotobuf.so",
    ],
    hdrs = glob(["include/google/protobuf/**/*.h"]),
    linkopts = [
        "-lprotobuf",
    ],
)
