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
import "private"

/*!
    \qmltype HueSaturation
    \inqmlmodule QtGraphicalEffects
    \since QtGraphicalEffects 1.0
    \inherits QtQuick2::Item
    \ingroup qtgraphicaleffects-color
    \brief Alters the source item colors in the HSL color space.

    HueSaturation is similar to the \l{QtGraphicalEffects1::Colorize}{Colorize}
    effect, but the hue and saturation property values are handled differently.
    The HueSaturation effect always shifts the hue, saturation, and lightness
    from the original, instead of setting them.

    \table
    \header
        \li Source
        \li Effect applied
    \row
        \li \image Original_bug.png
        \li \image HueSaturation_bug.png
    \endtable

    \section1 Example

    The following example shows how to apply the effect.
    \snippet HueSaturation-example.qml example

*/
Item {
    id: rootItem

    /*!
        This property defines the source item that provides the source pixels
        for the effect.

        \note It is not supported to let the effect include itself, for
        instance by setting source to the effect's parent.
    */
    property variant source: 0

    /*!
        This property defines the hue value which is added to the source hue
        value.

        The value ranges from -1.0 (decrease) to 1.0 (increase). By default, the
        property is set to \c 0.0 (no change).

        \table
        \header
        \li Output examples with different hue values
        \li
        \li
        \row
            \li \image HueSaturation_hue1.png
            \li \image HueSaturation_hue2.png
            \li \image HueSaturation_hue3.png
        \row
            \li \b { hue: -0.3 }
            \li \b { hue: 0.0 }
            \li \b { hue: 0.3 }
        \row
            \li \l saturation: 0
            \li \l saturation: 0
            \li \l saturation: 0
        \row
            \li \l lightness: 0
            \li \l lightness: 0
            \li \l lightness: 0
        \endtable

    */
    property real hue: 0.0

    /*!
        This property defines the saturation value value which is added to the
        source saturation value.

        The value ranges from -1.0 (decrease) to 1.0 (increase). By default, the
        property is set to \c 0.0 (no change).

        \table
        \header
        \li Output examples with different saturation values
        \li
        \li
        \row
            \li \image HueSaturation_saturation1.png
            \li \image HueSaturation_saturation2.png
            \li \image HueSaturation_saturation3.png
        \row
            \li \b { saturation: -0.8 }
            \li \b { saturation: 0.0 }
            \li \b { saturation: 1.0 }
        \row
            \li \l hue: 0
            \li \l hue: 0
            \li \l hue: 0
        \row
            \li \l lightness: 0
            \li \l lightness: 0
            \li \l lightness: 0
        \endtable

    */
    property real saturation: 0.0

    /*!
        This property defines the lightness value which is added to the source
        saturation value.

        The value ranges from -1.0 (decrease) to 1.0 (increase). By default, the
        property is set to \c 0.0 (no change).

        \table
        \header
        \li Output examples with different lightness values
        \li
        \li
        \row
            \li \image HueSaturation_lightness1.png
            \li \image HueSaturation_lightness2.png
            \li \image HueSaturation_lightness3.png
        \row
            \li \b { lightness: -0.5 }
            \li \b { lightness: 0.0 }
            \li \b { lightness: 0.5 }
        \row
            \li \l hue: 0
            \li \l hue: 0
            \li \l hue: 0
        \row
            \li \l saturation: 0
            \li \l saturation: 0
            \li \l saturation: 0
        \endtable

    */
    property real lightness: 0.0

    /*!
        This property allows the effect output pixels to be cached in order to
        improve the rendering performance.

        Every time the source or effect properties are changed, the pixels in
        the cache must be updated. Memory consumption is increased, because an
        extra buffer of memory is required for storing the effect output.

        It is recommended to disable the cache when the source or the effect
        properties are animated.

        By default, the property is set to \c false.
    */
    property bool cached: false

    SourceProxy {
        id: sourceProxy
        input: rootItem.source
    }

    ShaderEffectSource {
        id: cacheItem
        anchors.fill: parent
        visible: rootItem.cached
        smooth: true
        sourceItem: shaderItem
        live: true
        hideSource: visible
    }

    ShaderEffect {
        id: shaderItem
        property variant source: sourceProxy.output
        property variant hsl: Qt.vector3d(rootItem.hue, rootItem.saturation, rootItem.lightness)

        anchors.fill: parent

        fragmentShader: "
            varying highp vec2 qt_TexCoord0;
            uniform highp float qt_Opacity;
            uniform highp sampler2D source;
            uniform highp vec3 hsl;

            highp vec3 RGBtoHSL(highp vec3 color) {
                highp float cmin = min(color.r, min(color.g, color.b));
                highp float cmax = max(color.r, max(color.g, color.b));
                highp float h = 0.0;
                highp float s = 0.0;
                highp float l = (cmin + cmax) / 2.0;
                highp float diff = cmax - cmin;

                if (diff > 1.0 / 256.0) {
                    if (l < 0.5)
                        s = diff / (cmin + cmax);
                    else
                        s = diff / (2.0 - (cmin + cmax));

                    if (color.r == cmax)
                        h = (color.g - color.b) / diff;
                    else if (color.g == cmax)
                        h = 2.0 + (color.b - color.r) / diff;
                    else
                        h = 4.0 + (color.r - color.g) / diff;

                    h /= 6.0;
                }
                return vec3(h, s, l);
            }

            highp float hueToIntensity(highp float v1, highp float v2, highp float h) {
                h = fract(h);
                if (h < 1.0 / 6.0)
                    return v1 + (v2 - v1) * 6.0 * h;
                else if (h < 1.0 / 2.0)
                    return v2;
                else if (h < 2.0 / 3.0)
                    return v1 + (v2 - v1) * 6.0 * (2.0 / 3.0 - h);

                return v1;
            }

            highp vec3 HSLtoRGB(highp vec3 color) {
                highp float h = color.x;
                highp float l = color.z;
                highp float s = color.y;

                if (s < 1.0 / 256.0)
                    return vec3(l);

                highp float v1;
                highp float v2;
                if (l < 0.5)
                    v2 = l * (1.0 + s);
                else
                    v2 = (l + s) - (s * l);

                v1 = 2.0 * l - v2;

                highp float d = 1.0 / 3.0;
                highp float r = hueToIntensity(v1, v2, h + d);
                highp float g = hueToIntensity(v1, v2, h);
                highp float b = hueToIntensity(v1, v2, h - d);
                return vec3(r, g, b);
            }

            void main() {
                lowp vec4 sample = texture2D(source, qt_TexCoord0);
                sample = vec4(sample.rgb / max(1.0/256.0, sample.a), sample.a);
                sample.rgb = mix(vec3(dot(sample.rgb, vec3(0.2125, 0.7154, 0.0721))), sample.rgb, 1.0 + hsl.y);
                sample.xyz = RGBtoHSL(sample.rgb);
                sample.rgb = HSLtoRGB(vec3(sample.x + hsl.x, sample.y, sample.z));
                highp float c = step(0.0, hsl.z);
                sample.rgb = mix(sample.rgb, vec3(c), abs(hsl.z));
                gl_FragColor = vec4(sample.rgb * sample.a, sample.a) * qt_Opacity;
            }
        "
    }
}
