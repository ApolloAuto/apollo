package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "osqp",
    hdrs = [
        "include/auxil.h",
        "include/constants.h",
        "include/cs.h",
        "include/ctrlc.h",
        "include/glob_opts.h",
        "include/kkt.h",
        "include/lin_alg.h",
        "include/lin_sys.h",
        "include/osqp.h",
        "include/osqp_configure.h",
        "include/polish.h",
        "include/proj.h",
        "include/scaling.h",
        "include/types.h",
        "include/util.h"
    ],
    srcs = [ "libosqp.so" ],
    copts = [ "-fPIC"],
    include_prefix = "osqp",
    linkopts = [
        "-Wl,-rpath,/usr/lib/x86_64-linux-gnu/",
    ],
)

