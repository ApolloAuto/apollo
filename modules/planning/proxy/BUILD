package(default_visibility = ["//visibility:public"])

cc_library(
    name = "proxy",
    srcs = [
        "routing_proxy.cc",
    ],
    hdrs = glob([
        "*.h",
    ]),
    data = ["//modules/map:map_data"],
    deps = [
        "//external:gflags",
        "//modules/common/math",
        "//modules/common/status",
        "//modules/common/util",
        "//modules/map/proto:map_proto",
    ],
)
