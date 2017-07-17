package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "ros_common",
    srcs = [
        "lib/libcpp_common.so",
        "lib/libfastcdr.so",
        "lib/libfastrtps.so",
        "lib/librosconsole.so",
        "lib/librosconsole_backend_interface.so",
        "lib/librosconsole_print.so",
        "lib/libroscpp.so",
        "lib/libroscpp_serialization.so",
        "lib/librostime.so",
        "lib/libxmlrpcpp.so",
        "lib/librosconsole_log4cxx.so",
    ],
    hdrs = glob([
        "include/*/*.h",
    ]),
    includes = ["include"],
    linkopts = [
        "-lrt",
        "-lboost_system",
    ],
    include_prefix = "ros",
)
