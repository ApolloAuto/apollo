# Copyright 2018 The Cartographer Authors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Description:
#   Eigen is a C++ template library for linear algebra: vectors,
#   matrices, and related algorithms.

load("@rules_cc//cc:defs.bzl", "cc_library")

licenses([
    # Note: Eigen is an MPL2 library that includes GPL v3 and LGPL v2.1+ code.
    #       We've taken special care to not reference any restricted code.
    "reciprocal",  # MPL2
    "notice",  # Portions BSD
])

exports_files(["COPYING.MPL2"])

EIGEN_FILES = [
    "Eigen/**",
    "unsupported/Eigen/CXX11/**",
    "unsupported/Eigen/FFT",
    "unsupported/Eigen/**",
    "unsupported/Eigen/KroneckerProduct",
    "unsupported/Eigen/src/FFT/**",
    "unsupported/Eigen/src/KroneckerProduct/**",
    "unsupported/Eigen/MatrixFunctions",
    "unsupported/Eigen/SpecialFunctions",
    "unsupported/Eigen/src/MatrixFunctions/**",
    "unsupported/Eigen/src/SpecialFunctions/**",
]

# List of files picked up by glob but actually part of another target.
EIGEN_EXCLUDE_FILES = [
    "Eigen/src/Core/arch/AVX/PacketMathGoogleTest.cc",
]

EIGEN_MPL2_HEADER_FILES = glob(
    EIGEN_FILES,
    exclude = EIGEN_EXCLUDE_FILES,
)

cc_library(
    name = "eigen",
    hdrs = EIGEN_MPL2_HEADER_FILES,
    includes = ["."],
    visibility = ["//visibility:public"],
)
