# ----------------- BEGIN LICENSE BLOCK ---------------------------------
#
# INTEL CONFIDENTIAL
#
# Copyright (c) 2018 Intel Corporation
#
# This software and the related documents are Intel copyrighted materials, and
# your use of them is governed by the express license under which they were
# provided to you (License). Unless the License provides otherwise, you may not
# use, modify, copy, publish, distribute, disclose or transmit this software or
# the related documents without Intel's prior written permission.
#
# This software and the related documents are provided as is, with no express or
# implied warranties, other than those that are expressly stated in the License.
#
# ----------------- END LICENSE BLOCK -----------------------------------
##

package(
    default_visibility = ["//visibility:public"],
)

licenses(["notice"])


cc_library(
    name = "ad_rss",
    srcs = [
        "src/generated/physics/Acceleration.cpp",
        "src/generated/physics/CoordinateSystemAxis.cpp",
        "src/generated/physics/Distance.cpp",
        "src/generated/physics/DistanceSquared.cpp",
        "src/generated/physics/Duration.cpp",
        "src/generated/physics/DurationSquared.cpp",
        "src/generated/physics/ParametricValue.cpp",
        "src/generated/physics/Speed.cpp",
        "src/generated/physics/SpeedSquared.cpp",
        "src/generated/situation/LateralRelativePosition.cpp",
        "src/generated/situation/LongitudinalRelativePosition.cpp",
        "src/generated/situation/SituationType.cpp",
        "src/generated/state/LateralResponse.cpp",
        "src/generated/state/LongitudinalResponse.cpp",
        "src/generated/world/LaneDrivingDirection.cpp",
        "src/generated/world/LaneSegmentType.cpp",
        "src/generated/world/ObjectType.cpp",
        "src/core/RssCheck.cpp",
        "src/core/RssResponseResolving.cpp",
        "src/core/RssResponseTransformation.cpp",
        "src/core/RssSituationChecking.cpp",
        "src/core/RssSituationExtraction.cpp",
        "src/physics/Math.cpp",
        "src/situation/RSSFormulas.cpp",
        "src/situation/RssIntersectionChecker.cpp",
        "src/situation/RSSSituation.cpp",
        "src/world/RssSituationCoordinateSystemConversion.cpp",
        "src/world/RssObjectPositionExtractor.cpp",
    ] + glob(["src/**/*.hpp"]),
    hdrs = glob(["include/**/*.hpp"]),
    copts = ["-fPIC","-std=c++11","-Werror","-Wall","-Wextra","-pedantic","-Wconversion", "-Wsign-conversion", "--coverage"],
    includes = ["include", "include/generated", "src", "tests/test_support"], 
    linkopts = ["--coverage"],
)

################################################################################
# Install section
################################################################################

################################################################################
# Doxygen documentation
################################################################################
