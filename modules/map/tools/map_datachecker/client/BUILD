load("//tools:cpplint.bzl", "cpplint")


package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "map_datachecker_client",
    deps = [
        ":map_datachecker_client_lib",
    ],
)

cc_library(
    name = "map_datachecker_client_lib",
    srcs = glob(["*.cc"]),
    hdrs = glob(["*.h", "*.hpp"]),
    deps = [
        "//modules/map/tools/map_datachecker/proto:map_datachecker_proto",
        "//modules/map/tools/map_datachecker/proto:map_datachecker_grpc",
        "//modules/drivers/gnss/proto:gnss_proto",
        "//cyber:cyber",
        "//cyber/common:time_conversion",
        "//external:gflags",
        "@com_google_protobuf//:protobuf",
        "@proj4",
        "@yaml_cpp//:yaml",
    ],
    linkopts = [
        "-lboost_filesystem",
        "-lgrpc++",
    ],
)

cpplint()