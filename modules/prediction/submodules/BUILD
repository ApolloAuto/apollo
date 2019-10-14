load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "container_submodule_lib",
    srcs = ["container_submodule.cc"],
    hdrs = [
        "container_submodule.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"container_submodule\\\"",
    ],
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/adapters/proto:adapter_config_proto",
        "//modules/common/time:time",
        "//modules/perception/proto:perception_proto",
        "//modules/prediction/common:message_process",
        "//modules/prediction/common:prediction_gflags",
        "//modules/prediction/container:container_manager",
        "//modules/prediction/container/obstacles:obstacles_container",
    ],
)

cc_binary(
    name = "container_submodule.so",
    linkshared = True,
    linkstatic = False,
    deps = [":container_submodule_lib"],
)

cpplint()
