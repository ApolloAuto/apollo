package(default_visibility = ["//visibility:public"])

cc_library(
    name = "optimizer",
    srcs = [
        "optimizer.cc",
    ],
    hdrs = [
        "optimizer.h",
    ],
    deps = [
        "//modules/common/proto:error_code_proto",
    ],
)

cc_library(
    name = "path_optimizer",
    srcs = [
        "path_optimizer.cc",
    ],
    hdrs = [
        "path_optimizer.h",
    ],
    deps = [
        "@eigen//:eigen",
        "//modules/common/proto:error_code_proto",
        "//modules/planning/optimizer",
        "//modules/planning/optimizer/dp_poly_path",
    ],
)

cc_library(
    name = "dp_poly_path_optimizer",
    srcs = [
        "dp_poly_path_optimizer.cc",
    ],
    hdrs = [
        "dp_poly_path_optimizer.h",
    ],
    deps = [
        "//modules/planning/optimizer",
        "//modules/planning/optimizer:path_optimizer",
    ],
)

cc_library(
    name = "speed_optimizer",
    srcs = [
        "speed_optimizer.cc",
    ],
    hdrs = [
        "speed_optimizer.h",
        "optimizer.h",
    ],
    deps = [
        "@eigen//:eigen",
        "//modules/common/proto:error_code_proto",
        "//modules/planning/common:data_center",
        "//modules/planning/common:decision_data",
        "//modules/planning/common:planning_data",
        "//modules/planning/common/path:path_data",
        "//modules/planning/common/speed:speed_data",
    ],
)
