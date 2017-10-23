cc_library(
    name = "dynamicuda",
    hdrs = glob([
        "modules/dynamicuda/include/**/*.hpp",
    ]),
    includes = [
        "modules/dynamicuda/include",
    ],
)

cc_library(
    name = "core",
    srcs = glob(["modules/core/src/**/*.cpp"]),
    hdrs = glob([
        "modules/core/src/**/*.hpp",
        "modules/core/include/**/*.hpp",
    ]) + [
        ":module_includes",
        ":cvconfig",
        ":version_string",
    ],
    copts = [
        "-Imodules/dynamicuda/include",
    ],
    includes = [
        "modules/core/include",
    ],
    # Note that opencv core requires zlib and pthread to build.
    linkopts = [
        "-pthread",
        "-lz",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":dynamicuda",
    ],
)

cc_library(
    name = "imgproc",
    srcs = glob(["modules/imgproc/src/**/*.cpp"]),
    hdrs = glob([
        "modules/imgproc/src/**/*.hpp",
        "modules/imgproc/src/**/*.h",
        "modules/imgproc/include/**/*.hpp",
        "modules/imgproc/include/**/*.h",
    ]) + [
        ":module_includes",
    ],
    includes = [
        "modules/imgproc/include",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":core",
    ],
)

cc_library(
    name = "ml",
    srcs = glob(["modules/ml/src/**/*.cpp"]),
    hdrs = glob([
        "modules/ml/src/**/*.hpp",
        "modules/ml/src/**/*.h",
        "modules/ml/include/**/*.hpp",
        "modules/ml/include/**/*.h",
    ]) + [
        ":module_includes",
    ],
    includes = [
        "modules/ml/include",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":core",
    ],
)

cc_library(
    name = "video",
    srcs = glob(["modules/video/src/**/*.cpp"]),
    hdrs = glob([
        "modules/video/src/**/*.hpp",
        "modules/video/src/**/*.h",
        "modules/video/include/**/*.hpp",
        "modules/video/include/**/*.h",
    ]) + [
        ":module_includes",
    ],
    includes = [
        "modules/video/include",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":core",
    ],
)

cc_library(
    name = "highgui",
    srcs = glob(
        [
            "modules/highgui/src/**/*.cpp",
        ],
        exclude = [
            "modules/highgui/src/cap*.cpp",
            "modules/highgui/src/window_carbon.cpp",
        ],
    ),
    hdrs = glob([
        "modules/highgui/src/**/*.hpp",
        "modules/highgui/src/**/*.h",
        "modules/highgui/include/**/*.hpp",
        "modules/highgui/include/**/*.h",
    ]) + [
        ":module_includes",
    ],
    includes = [
        "modules/highgui/include",
    ],
    linkopts = [
        "-ljpeg",
        "-ltiff",
        "-lpng",
        "-ljasper",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":core",
        ":imgproc",
    ],
)

genrule(
    name = "module_includes",
    outs = ["opencv2/opencv_modules.hpp"],
    cmd = """
cat > $@ <<"EOF"
#define HAVE_OPENCV_CORE
#define HAVE_OPENCV_IMGPROC
#define HAVE_OPENCV_ML
EOF""",
)

genrule(
    name = "cvconfig",
    outs = ["cvconfig.h"],
    cmd = """
cat > $@ <<"EOF"
// JPEG-2000
#define HAVE_JASPER

// IJG JPEG
#define HAVE_JPEG

// PNG
#define HAVE_PNG

// TIFF
#define HAVE_TIFF

// Compile for 'real' NVIDIA GPU architectures
#define CUDA_ARCH_BIN ""

// NVIDIA GPU features are used
#define CUDA_ARCH_FEATURES ""

// Compile for 'virtual' NVIDIA PTX architectures
#define CUDA_ARCH_PTX ""
EOF""",
)

genrule(
    name = "version_string",
    outs = ["version_string.inc"],
    cmd = """
cat > $@ <<"EOF"
"\\n"
"General configuration for OpenCV 2.4.13.2 =====================================\\n"
"  Version control:               unknown\\n"
"\\n"
"  Platform:\\n"
"    Host:                        Linux 4.4.0-53-generic x86_64\\n"
"    CMake:                       3.5.1\\n"
"    CMake generator:             Unix Makefiles\\n"
"    CMake build tool:            /usr/bin/make\\n"
"    Configuration:               Release\\n"
"\\n"
"  C/C++:\\n"
"    Built as dynamic libs?:      YES\\n"
"    C++ Compiler:                /usr/bin/c++  (ver 5.4.0)\\n"
"    C++ flags (Release):         -fsigned-char -W -Wall -Werror=return-type -Werror=address -Werror=sequence-point -Wformat -Werror=format-security -Wmissing-declarations -Wundef -Winit-self -Wpointer-arith -Wshadow -Wsign-promo -Wno-narrowing -Wno-delete-non-virtual-dtor -Wno-comment -Wno-array-bounds -Wno-aggressive-loop-optimizations -fdiagnostics-show-option -Wno-long-long -pthread -fomit-frame-pointer -msse -msse2 -msse3 -ffunction-sections -O3 -DNDEBUG  -DNDEBUG\\n"
"    C++ flags (Debug):           -fsigned-char -W -Wall -Werror=return-type -Werror=address -Werror=sequence-point -Wformat -Werror=format-security -Wmissing-declarations -Wundef -Winit-self -Wpointer-arith -Wshadow -Wsign-promo -Wno-narrowing -Wno-delete-non-virtual-dtor -Wno-comment -Wno-array-bounds -Wno-aggressive-loop-optimizations -fdiagnostics-show-option -Wno-long-long -pthread -fomit-frame-pointer -msse -msse2 -msse3 -ffunction-sections -g  -O0 -DDEBUG -D_DEBUG\\n"
"    C Compiler:                  /usr/bin/cc\\n"
"    C flags (Release):           -fsigned-char -W -Wall -Werror=return-type -Werror=address -Werror=sequence-point -Wformat -Werror=format-security -Wmissing-declarations -Wmissing-prototypes -Wstrict-prototypes -Wundef -Winit-self -Wpointer-arith -Wshadow -Wno-narrowing -Wno-comment -Wno-array-bounds -Wno-aggressive-loop-optimizations -fdiagnostics-show-option -Wno-long-long -pthread -fomit-frame-pointer -msse -msse2 -msse3 -ffunction-sections -O3 -DNDEBUG  -DNDEBUG\\n"
"    C flags (Debug):             -fsigned-char -W -Wall -Werror=return-type -Werror=address -Werror=sequence-point -Wformat -Werror=format-security -Wmissing-declarations -Wmissing-prototypes -Wstrict-prototypes -Wundef -Winit-self -Wpointer-arith -Wshadow -Wno-narrowing -Wno-comment -Wno-array-bounds -Wno-aggressive-loop-optimizations -fdiagnostics-show-option -Wno-long-long -pthread -fomit-frame-pointer -msse -msse2 -msse3 -ffunction-sections -g  -O0 -DDEBUG -D_DEBUG\\n"
"    Linker flags (Release):\\n"
"    Linker flags (Debug):\\n"
"    ccache:                      YES\\n"
"    Precompiled headers:         NO\\n"
"\\n"
"  OpenCV modules:\\n"
"    To be built:                 core flann imgproc highgui features2d calib3d ml video legacy objdetect photo gpu ocl nonfree contrib stitching superres ts videostab\\n"
"    Disabled:                    world\\n"
"    Disabled by dependency:      -\\n"
"    Unavailable:                 androidcamera dynamicuda java python viz\\n"
"\\n"
"  GUI: \\n"
"    QT:                          NO\\n"
"    GTK+ 2.x:                    NO\\n"
"    GThread :                    NO\\n"
"    GtkGlExt:                    NO\\n"
"    OpenGL support:              NO\\n"
"    VTK support:                 NO\\n"
"\\n"
"  Media I/O: \\n"
"    ZLib:                        /usr/lib/x86_64-linux-gnu/libz.so (ver 1.2.8)\\n"
"    JPEG:                        libjpeg (ver 62)\\n"
"    PNG:                         build (ver 1.5.27)\\n"
"    TIFF:                        build (ver 42 - 4.0.2)\\n"
"    JPEG 2000:                   build (ver 1.900.1)\\n"
"    OpenEXR:                     build (ver 1.7.1)\\n"
"\\n"
"  Video I/O:\\n"
"    DC1394 1.x:                  NO\\n"
"    DC1394 2.x:                  NO\\n"
"    FFMPEG:                      NO\\n"
"      avcodec:                   NO\\n"
"      avformat:                  NO\\n"
"      avutil:                    NO\\n"
"      swscale:                   NO\\n"
"      avresample:                NO\\n"
"    GStreamer:                   NO\\n"
"    OpenNI:                      NO\\n"
"    OpenNI PrimeSensor Modules:  NO\\n"
"    PvAPI:                       NO\\n"
"    GigEVisionSDK:               NO\\n"
"    UniCap:                      NO\\n"
"    UniCap ucil:                 NO\\n"
"    V4L/V4L2:                    NO/YES\\n"
"    XIMEA:                       NO\\n"
"    Xine:                        NO\\n"
"\\n"
"  Other third-party libraries:\\n"
"    Use IPP:                     NO\\n"
"    Use Eigen:                   YES (ver 3.2.92)\\n"
"    Use TBB:                     NO\\n"
"    Use OpenMP:                  NO\\n"
"    Use GCD                      NO\\n"
"    Use Concurrency              NO\\n"
"    Use C=:                      NO\\n"
"    Use Cuda:                    NO\\n"
"    Use OpenCL:                  YES\\n"
"\\n"
"  OpenCL:\\n"
"    Version:                     dynamic\\n"
"    Include path:                /home/breakds/Downloads/opencv-2.4.13.2/3rdparty/include/opencl/1.2\\n"
"    Use AMD FFT:                 NO\\n"
"    Use AMD BLAS:                NO\\n"
"\\n"
"  Python:\\n"
"    Interpreter:                 /usr/bin/python2 (ver 2.7.12)\\n"
"\\n"
"  Java:\\n"
"    ant:                         NO\\n"
"    JNI:                         NO\\n"
"    Java tests:                  NO\\n"
"\\n"
"  Documentation:\\n"
"    Build Documentation:         NO\\n"
"    Sphinx:                      NO\\n"
"    PdfLaTeX compiler:           NO\\n"
"    Doxygen:                     NO\\n"
"\\n"
"  Tests and samples:\\n"
"    Tests:                       YES\\n"
"    Performance tests:           YES\\n"
"    C/C++ Examples:              NO\\n"
"\\n"
"  Install path:                  /usr/local\\n"
"\\n"
"  cvconfig.h is in:              /home/breakds/Downloads/opencv-2.4.13.2/build\\n"
"-----------------------------------------------------------------\\n"
"\\n"
EOF""",
)
