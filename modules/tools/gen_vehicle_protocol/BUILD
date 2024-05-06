load("//tools/proto:proto.bzl", "apollo_py_binary")
# load("//tools/install:install.bzl", "install")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

apollo_py_binary(
    name = "extract_dbc_meta",
    srcs = ["extract_dbc_meta.py"],
)

apollo_py_binary(
    name = "gen",
    srcs = ["gen.py"],
    deps = [
        ":extract_dbc_meta",
        ":gen_canbus_conf",
        ":gen_proto_file",
        ":gen_protocols",
        ":gen_vehicle_controller_and_manager",
    ],
)

apollo_py_binary(
    name = "gen_canbus_conf",
    srcs = ["gen_canbus_conf.py"],
)

apollo_py_binary(
    name = "gen_proto_file",
    srcs = ["gen_proto_file.py"],
)

apollo_py_binary(
    name = "gen_protocols",
    srcs = ["gen_protocols.py"],
)

apollo_py_binary(
    name = "gen_vehicle_controller_and_manager",
    srcs = ["gen_vehicle_controller_and_manager.py"],
)

# install(
#     name = "install",
#     py_dest = "tools/gen_vehicle_protocol",
#     data_dest = "tools/gen_vehicle_protocol",
#     data = [":data"],
#     targets = [
#         ":gen",
#     ]

# )

filegroup(
    name = "data",
    srcs = glob([
        "*.yml",
    ]) + ["README.md"] + glob(["template/*"]),
)

apollo_package()