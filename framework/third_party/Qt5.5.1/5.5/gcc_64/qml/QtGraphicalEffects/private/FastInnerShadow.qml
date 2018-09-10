/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the Qt Graphical Effects module.
**
** $QT_BEGIN_LICENSE:BSD$
** You may use this file under the terms of the BSD license as follows:
**
** "Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are
** met:
**   * Redistributions of source code must retain the above copyright
**     notice, this list of conditions and the following disclaimer.
**   * Redistributions in binary form must reproduce the above copyright
**     notice, this list of conditions and the following disclaimer in
**     the documentation and/or other materials provided with the
**     distribution.
**   * Neither the name of The Qt Company Ltd nor the names of its
**     contributors may be used to endorse or promote products derived
**     from this software without specific prior written permission.
**
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
**
** $QT_END_LICENSE$
**
****************************************************************************/

import QtQuick 2.0

Item {
    id: rootItem
    property variant source
    property real blur: 0.0
    property real horizontalOffset: 0
    property real verticalOffset: 0
    property real spread: 0.0
    property color color: "black"
    property bool cached: false

    SourceProxy {
        id: sourceProxy
        input: rootItem.source
    }

    ShaderEffectSource {
        id: cacheItem
        anchors.fill: shaderItem
        visible: rootItem.cached
        smooth: true
        sourceItem: shaderItem
        live: true
        hideSource: visible
    }

    property string __internalBlurVertexShader: "
        attribute highp vec4 qt_Vertex;
        attribute highp vec2 qt_MultiTexCoord0;
        uniform highp mat4 qt_Matrix;
        uniform highp float yStep;
        uniform highp float xStep;
        varying highp vec2 qt_TexCoord0;
        varying highp vec2 qt_TexCoord1;
        varying highp vec2 qt_TexCoord2;
        varying highp vec2 qt_TexCoord3;

        void main() {
            qt_TexCoord0 = vec2(qt_MultiTexCoord0.x + xStep, qt_MultiTexCoord0.y + yStep * 0.36);
            qt_TexCoord1 = vec2(qt_MultiTexCoord0.x + xStep * 0.36, qt_MultiTexCoord0.y - yStep);
            qt_TexCoord2 = vec2(qt_MultiTexCoord0.x - xStep * 0.36, qt_MultiTexCoord0.y + yStep);
            qt_TexCoord3 = vec2(qt_MultiTexCoord0.x - xStep, qt_MultiTexCoord0.y - yStep * 0.36);
            gl_Position = qt_Matrix * qt_Vertex;
        }
    "

    property string __internalBlurFragmentShader: "
        uniform lowp sampler2D source;
        uniform lowp float qt_Opacity;
        varying highp vec2 qt_TexCoord0;
        varying highp vec2 qt_TexCoord1;
        varying highp vec2 qt_TexCoord2;
        varying highp vec2 qt_TexCoord3;

        void main() {
            highp vec4 sourceColor = (texture2D(source, qt_TexCoord0) +
            texture2D(source, qt_TexCoord1) +
            texture2D(source, qt_TexCoord2) +
            texture2D(source, qt_TexCoord3)) * 0.25;
            gl_FragColor = sourceColor * qt_Opacity;
        }
   "

    ShaderEffect {
        id: level0
        property variant source: sourceProxy.output
        property real horizontalOffset: rootItem.horizontalOffset / rootItem.width
        property real verticalOffset: rootItem.verticalOffset / rootItem.width
        property color color: rootItem.color

        anchors.fill: parent
        visible: false
        smooth: true
        fragmentShader: "
            varying highp vec2 qt_TexCoord0;
            uniform lowp float qt_Opacity;
            uniform highp sampler2D source;
            uniform lowp vec4 color;
            uniform highp float horizontalOffset;
            uniform highp float verticalOffset;

            void main(void) {
                highp vec2 pos = qt_TexCoord0 - vec2(horizontalOffset, verticalOffset);
                lowp float ea = step(0.0, pos.x) * step(0.0, pos.y) * step(pos.x, 1.0) * step(pos.y, 1.0);
                lowp float eb = 1.0 - ea;
                gl_FragColor = (eb * color + ea * color * (1.0 - texture2D(source, pos).a)) * qt_Opacity;
            }
        "
    }

    ShaderEffectSource {
        id: level1
        width: Math.ceil(shaderItem.width / 32) * 32
        height: Math.ceil(shaderItem.height / 32) * 32
        sourceItem: level0
        hideSource: rootItem.visible
        smooth: true
        visible: false
    }

    ShaderEffect {
        id: effect1
        property variant source: level1
        property real yStep: 1/height
        property real xStep: 1/width
        anchors.fill: level2
        visible: false
        smooth: true
        vertexShader: __internalBlurVertexShader
        fragmentShader: __internalBlurFragmentShader
    }

    ShaderEffectSource {
        id: level2
        width: level1.width / 2
        height: level1.height / 2
        sourceItem: effect1
        hideSource: rootItem.visible
        visible: false
        smooth: true
    }

    ShaderEffect {
        id: effect2
        property variant source: level2
        property real yStep: 1/height
        property real xStep: 1/width
        anchors.fill: level3
        visible: false
        smooth: true
        vertexShader: __internalBlurVertexShader
        fragmentShader: __internalBlurFragmentShader
    }

    ShaderEffectSource {
        id: level3
        width: level2.width / 2
        height: level2.height / 2
        sourceItem: effect2
        hideSource: rootItem.visible
        visible: false
        smooth: true
    }

    ShaderEffect {
        id: effect3
        property variant source: level3
        property real yStep: 1/height
        property real xStep: 1/width
        anchors.fill: level4
        visible: false
        smooth: true
        vertexShader: __internalBlurVertexShader
        fragmentShader: __internalBlurFragmentShader
    }

    ShaderEffectSource {
        id: level4
        width: level3.width / 2
        height: level3.height / 2
        sourceItem: effect3
        hideSource: rootItem.visible
        visible: false
        smooth: true
    }

    ShaderEffect {
        id: effect4
        property variant source: level4
        property real yStep: 1/height
        property real xStep: 1/width
        anchors.fill: level5
        visible: false
        smooth: true
        vertexShader: __internalBlurVertexShader
        fragmentShader: __internalBlurFragmentShader
    }

    ShaderEffectSource {
        id: level5
        width: level4.width / 2
        height: level4.height / 2
        sourceItem: effect4
        hideSource: rootItem.visible
        visible: false
        smooth: true
    }

    ShaderEffect {
        id: effect5
        property variant source: level5
        property real yStep: 1/height
        property real xStep: 1/width
        anchors.fill: level6
        visible: false
        smooth: true
        vertexShader: __internalBlurVertexShader
        fragmentShader: __internalBlurFragmentShader
    }

    ShaderEffectSource {
        id: level6
        width: level5.width / 2
        height: level5.height / 2
        sourceItem: effect5
        hideSource: rootItem.visible
        visible: false
        smooth: true
    }

    Item {
        id: dummysource
        width: 1
        height: 1
        visible: false
    }

    ShaderEffectSource {
        id: dummy
        width: 1
        height: 1
        sourceItem: dummysource
        visible: false
        smooth: false
        live: false
    }

    ShaderEffect {
        id: shaderItem
        width: parent.width
        height: parent.height

        property variant original: sourceProxy.output
        property variant source1: level1
        property variant source2: level2
        property variant source3: level3
        property variant source4: level4
        property variant source5: level5
        property variant source6: level6
        property real lod: rootItem.blur

        property real weight1;
        property real weight2;
        property real weight3;
        property real weight4;
        property real weight5;
        property real weight6;

        property real spread: 1.0 - (rootItem.spread * 0.98)
        property color color: rootItem.color

        function weight(v) {
            if (v <= 0.0)
                return 1
            if (v >= 0.5)
                return 0

            return 1.0 - v / 0.5
        }

        function calculateWeights() {

            var w1 = weight(Math.abs(lod - 0.100))
            var w2 = weight(Math.abs(lod - 0.300))
            var w3 = weight(Math.abs(lod - 0.500))
            var w4 = weight(Math.abs(lod - 0.700))
            var w5 = weight(Math.abs(lod - 0.900))
            var w6 = weight(Math.abs(lod - 1.100))

            var sum = w1 + w2 + w3 + w4 + w5 + w6;
            weight1 = w1 / sum;
            weight2 = w2 / sum;
            weight3 = w3 / sum;
            weight4 = w4 / sum;
            weight5 = w5 / sum;
            weight6 = w6 / sum;

            upateSources()
        }

        function upateSources() {
            var sources = new Array();
            var weights = new Array();

            if (weight1 > 0) {
                sources.push(level1)
                weights.push(weight1)
            }

            if (weight2 > 0) {
                sources.push(level2)
                weights.push(weight2)
            }

            if (weight3 > 0) {
                sources.push(level3)
                weights.push(weight3)
            }

            if (weight4 > 0) {
                sources.push(level4)
                weights.push(weight4)
            }

            if (weight5 > 0) {
                sources.push(level5)
                weights.push(weight5)
            }

            if (weight6 > 0) {
                sources.push(level6)
                weights.push(weight6)
            }

            for (var j = sources.length; j < 6; j++) {
                sources.push(dummy)
                weights.push(0.0)
            }

            source1 = sources[0]
            source2 = sources[1]
            source3 = sources[2]
            source4 = sources[3]
            source5 = sources[4]
            source6 = sources[5]

            weight1 = weights[0]
            weight2 = weights[1]
            weight3 = weights[2]
            weight4 = weights[3]
            weight5 = weights[4]
            weight6 = weights[5]
        }

        Component.onCompleted: calculateWeights()

        onLodChanged: calculateWeights()

        fragmentShader: "
            uniform lowp sampler2D original;
            uniform lowp sampler2D source1;
            uniform lowp sampler2D source2;
            uniform lowp sampler2D source3;
            uniform lowp sampler2D source4;
            uniform lowp sampler2D source5;
            uniform mediump float weight1;
            uniform mediump float weight2;
            uniform mediump float weight3;
            uniform mediump float weight4;
            uniform mediump float weight5;
            uniform highp vec4 color;
            uniform highp float spread;
            uniform lowp float qt_Opacity;
            varying mediump vec2 qt_TexCoord0;

            highp float linearstep(highp float e0, highp float e1, highp float x) {
                return clamp((x - e0) / (e1 - e0), 0.0, 1.0);
            }

            void main() {
                lowp vec4 shadowColor = texture2D(source1, qt_TexCoord0) * weight1;
                shadowColor += texture2D(source2, qt_TexCoord0) * weight2;
                shadowColor += texture2D(source3, qt_TexCoord0) * weight3;
                shadowColor += texture2D(source4, qt_TexCoord0) * weight4;
                shadowColor += texture2D(source5, qt_TexCoord0) * weight5;
                lowp vec4 originalColor = texture2D(original, qt_TexCoord0);
                shadowColor.rgb = mix(originalColor.rgb, color.rgb * originalColor.a, linearstep(0.0, spread, shadowColor.a));
                gl_FragColor = vec4(shadowColor.rgb, originalColor.a) * originalColor.a * qt_Opacity;
            }
        "
    }
}
