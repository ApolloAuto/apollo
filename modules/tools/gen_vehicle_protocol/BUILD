load("@rules_python//python:defs.bzl", "py_binary")

package(default_visibility = ["//visibility:public"])

py_binary(
    name = "extract_dbc_meta",
    srcs = ["extract_dbc_meta.py"],
)

py_binary(
    name = "gen",
    srcs = ["gen.py"],
    deps = [
        ":extract_dbc_meta",
        ":gen_proto_file",
        ":gen_protocols",
        ":gen_vehicle_controller_and_manager",
    ],
)

py_binary(
    name = "gen_proto_file",
    srcs = ["gen_proto_file.py"],
)

py_binary(
    name = "gen_protocols",
    srcs = ["gen_protocols.py"],
)

py_binary(
    name = "gen_vehicle_controller_and_manager",
    srcs = ["gen_vehicle_controller_and_manager.py"],
)
