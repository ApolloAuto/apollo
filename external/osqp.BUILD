package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_import(
    name = "osqp_lib",
    static_library = "osqp/libosqp.a",
    shared_library = "osqp/libosqp.so",
)

cc_library(
    name = "osqp",
    hdrs = glob(["osqp/include/*.h"]),
    deps = [
        ":osqp_lib",
    ],
)
