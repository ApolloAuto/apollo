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

cc_binary(
    name = "container_submodule.so",
    linkshared = True,
    linkstatic = False,
    deps = [":container_submodule_lib"],
)

cc_library(
    name = "scenario_submodule_lib",
    srcs = ["scenario_submodule.cc"],
    hdrs = [
        "scenario_submodule.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"scenario_submodule\\\"",
    ],
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/adapters/proto:adapter_config_proto",
        "//modules/common/time",
        "//modules/perception/proto:perception_proto",
        "//modules/prediction/common:message_process",
        "//modules/prediction/common:prediction_gflags",
        "//modules/prediction/proto:submodule_messages_proto",
        "//modules/prediction/scenario:scenario_manager",
    ],
)

cc_binary(
    name = "scenario_submodule.so",
    linkshared = True,
    linkstatic = False,
    deps = [":scenario_submodule_lib"],
)

cpplint()
