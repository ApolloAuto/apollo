package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "osqp",
    includes = [
	".",
    ],
    include_prefix = "osqp",
    linkopts = [
	"-L/usr/local/lib",
        "-Wl,-rpath,/usr/lib/x86_64-linux-gnu/",
	"-losqp",
    ],
)

