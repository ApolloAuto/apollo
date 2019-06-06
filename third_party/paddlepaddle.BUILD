package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "paddlepaddle",
    srcs = [
        "lib/libpaddle_fluid.a",
        "lib/libpaddle_fluid.so",
    ],
    hdrs = glob(["*.h"]),
    includes = ["include"],
    linkopts = [
      "-L/usr/local/apollo/libtorch/lib -lprotobuf",
    ],
)
