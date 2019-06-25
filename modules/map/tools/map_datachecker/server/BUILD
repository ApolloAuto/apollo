load("//tools:cpplint.bzl", "cpplint")


package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "map_datachecker_server",
    deps = [
        ":map_datachecker_server_lib",
    ],
)

cc_library(
    name = "map_datachecker_server_lib",
    srcs = glob(["*.cc"]),
    hdrs = glob(["*.h", "*.hpp"]),
    deps = [
        "//modules/map/tools/map_datachecker/proto:map_datachecker_proto",
        "//modules/map/tools/map_datachecker/proto:map_datachecker_grpc",
        "//modules/drivers/gnss/proto:gnss_proto",
        "//cyber:cyber",
        "//cyber/common:time_conversion",
        "@com_google_protobuf//:protobuf",
        "//external:gflags",
        "@proj4",
    ],
    linkopts = [
        "-lboost_filesystem",
        "-lgrpc++",
    ],
)

cpplint()