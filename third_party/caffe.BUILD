licenses(["notice"])

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "lib",
    includes = ["."],
    linkopts = [
        "-L/usr/local/cuda/lib64:/usr/lib/x86_64-linux-gnu/hdf5/serial/lib",
        "-Wl,-rpath,/usr/local/lib:/usr/lib/x86_64-linux-gnu/hdf5/serial/lib:/usr/local/cuda/lib64",
        "-lboost_system",
        "-lboost_thread",
        "-lboost_filesystem",
        "-lpthread",
        "-lblas",
        "-lcblas",
        "-lhdf5_hl",
        "-lhdf5",
        "-lz",
        "-ldl",
        "-lm",
        "-lopencv_core",
        "-lopencv_highgui",
        "-lopencv_imgproc",
        "-Wl,--whole-archive -lcaffe -Wl,--no-whole-archive",
    ],
)
