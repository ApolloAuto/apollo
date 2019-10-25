load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "container_submodule_lib",
    srcs = ["container_submodule.cc"],
    hdrs = [
        "container_submodule.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"prediction\\\"",
    ],
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/adapters/proto:adapter_config_proto",
        "//modules/common/time",
        "//modules/perception/proto:perception_proto",
        "//modules/prediction/common:message_process",
        "//modules/prediction/common:prediction_gflags",
        "//modules/prediction/container:container_manager",
        "//modules/prediction/container/obstacles:obstacles_container",
        "//modules/prediction/proto:submodule_messages_proto",
        "//modules/storytelling/proto:story_proto",
    ],
)

cc_library(
    name = "container_output",
    srcs = ["container_output.cc"],
    hdrs = ["container_output.h"],
    copts = [
        "-DMODULE_NAME=\\\"prediction\\\"",
    ],
    deps = [
        "//modules/common/util:lru_cache",
        "//modules/perception/proto:perception_proto",
        "//modules/prediction/common:prediction_gflags",
        "//modules/prediction/container/obstacles:obstacle",
    ],
)

cc_binary(
    name = "container_submodule.so",
    linkshared = True,
    linkstatic = False,
    deps = [":container_submodule_lib"],
)

cc_library(
    name = "evaluator_submodule_lib",
    srcs = ["evaluator_submodule.cc"],
    hdrs = [
        "evaluator_submodule.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"prediction\\\"",
    ],
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/adapters/proto:adapter_config_proto",
        "//modules/common/time",
        "//modules/perception/proto:perception_proto",
        "//modules/prediction/common:message_process",
        "//modules/prediction/common:prediction_gflags",
        "//modules/prediction/evaluator:evaluator_manager",
        "//modules/prediction/submodules:container_output",
    ],
)

cc_binary(
    name = "evaluator_submodule.so",
    linkshared = True,
    linkstatic = False,
    deps = [":evaluator_submodule_lib"],
)

cpplint()
