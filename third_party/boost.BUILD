licenses(["notice"])

package(default_visibility = ["//visibility:public"])

# TODO(storypku): remove this and use proto_boost instead.
cc_library(
    name = "boost",
    includes = ["."],
    linkopts = [
		"-L/opt/apollo/sysroot/lib",
        "-lboost_system",
        "-lboost_filesystem",
        "-lboost_program_options",
        "-lboost_thread",
    ],
)
