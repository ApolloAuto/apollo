package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "ros_common",
    srcs = [
        "lib/libactionlib.so",
        "lib/libcpp_common.so",
        "lib/libcv_bridge.so",
        "lib/libeigen_conversions.so",
        "lib/libfastcdr.so",
        "lib/libfastrtps.so",
        "lib/liborocos-kdl.so.1.3",
        "lib/librosbag.so",
        "lib/librosbag_storage.so",
        "lib/librosconsole.so",
        "lib/librosconsole_backend_interface.so",
        "lib/librosconsole_log4cxx.so",
        "lib/librosconsole_print.so",
        "lib/libroscpp.so",
        "lib/libroscpp_serialization.so",
        "lib/libroslz4.so",
        "lib/librostime.so",
        "lib/libtf2.so",
        "lib/libtf2_ros.so",
        "lib/libtopic_tools.so",
        "lib/libxmlrpcpp.so",
    ],
    hdrs = glob([
        "include/*/*.h",
    ]),
    include_prefix = "ros",
    includes = ["include"],
    linkopts = [
        "-lrt",
        "-lboost_system",
    ],
)
