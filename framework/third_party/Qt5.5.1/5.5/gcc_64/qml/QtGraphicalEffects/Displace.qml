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
    \qmltype Displace
    \inqmlmodule QtGraphicalEffects
    \since QtGraphicalEffects 1.0
    \inherits QtQuick2::Item
    \ingroup qtgraphicaleffects-distortion
    \brief Moves the pixels of the source item according to the given
    displacement map.

    \table
    \header
        \li Source
        \li DisplacementSource
        \li Effect applied
    \row
        \li \image Original_bug.png
        \li \image Displace_map.png
        \li \image Displace_bug.png
    \endtable

    \section1 Example

    The following example shows how to apply the effect.
    \snippet Displace-example.qml example

*/
Item {
    id: rootItem

    /*!
        This property defines the source item for the pixels that are going to
        be displaced according to the data from
        \l{Displace::displacementSource}{displacementSource}.

        \note It is not supported to let the effect include itself, for
        instance by setting source to the effect's parent.
    */
    property variant source

    /*!
        This property defines the item that is going to be used as the
        displacement map. The displacementSource item gets rendered into the
        intermediate pixel buffer. The red and green component values from the
        result determine the displacement of the pixels from the source item.

        The format for the displacement map is similar to the tangent space
        normal maps, which can be created with most 3D-modeling tools. Many
        image processing tools include the support for generating normal maps.
        Alternatively, the displacement map for this effect can also be a QML
        element which is colored appropriately. Like any QML element, it can be
        animated. It is recommended that the size of the diplacement map matches
        the size of the \l{Displace::source}{source}.

        The displace data is interpreted in the RGBA format. For every pixel:
        the red channel stores the x-axis displacement, and the green channel
        stores the y-axis displacement. Blue and alpha channels are ignored for
        this effect.

        Assuming that red channel value 1.0 is fully red (0.0 having no red at
        all), this effect considers pixel component value 0.5 to cause no
        displacement at all. Values above 0.5 shift pixels to the left, values
        below 0.5 do the shift to the right. In a similar way, green channel
        values above 0.5 displace the pixels upwards, and values below 0.5 shift
        the pixels downwards. The actual amount of displacement in pixels
        depends on the \l displacement property.

    */
    property variant displacementSource

    /*!
        This property defines the scale for the displacement. The bigger scale,
        the bigger the displacement of the pixels. The value set to 0.0 causes
        no displacement.

        The value ranges from -1.0 (inverted maximum shift, according to
        displacementSource) to 1.0 (maximum shift, according to
        displacementSource). By default, the property is set to \c 0.0 (no
        displacement).

        \table
        \header
        \li Output examples with different displacement values
        \li
        \li
        \row
            \li \image Displace_displacement1.png
            \li \image Displace_displacement2.png
            \li \image Displace_displacement3.png
        \row
            \li \b { displacement: -0.2 }
            \li \b { displacement: 0.0 }
            \li \b { displacement: 0.2 }
        \endtable

    */
    property real displacement: 0.0

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

    SourceProxy {
        id: displacementSourceProxy
        input: rootItem.displacementSource
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
        property variant displacementSource: displacementSourceProxy.output
        property real displacement: rootItem.displacement
        property real xPixel: 1.0/width
        property real yPixel: 1.0/height

        anchors.fill: parent

        fragmentShader: "
            varying highp vec2 qt_TexCoord0;
            uniform highp float qt_Opacity;
            uniform lowp sampler2D source;
            uniform lowp sampler2D displacementSource;
            uniform highp float displacement;
            uniform highp float xPixel;
            uniform highp float yPixel;

            highp float linearstep(highp float e0, highp float e1, highp float x) {
                return clamp((x - e0) / (e1 - e0), 0.0, 1.0);
            }

            void main() {
                lowp vec4 offset = texture2D(displacementSource, qt_TexCoord0);
                offset.xy -= vec2(0.5, 0.5);
                offset.xy = offset.xy * step(vec2(1.0/256.0), abs(offset.xy));
                highp vec2 tx = qt_TexCoord0 + (vec2(-offset.x, offset.y) * displacement);

                lowp float e1 = linearstep(0.0, xPixel, tx.x);
                lowp float e2 = linearstep(0.0, yPixel, tx.y);
                lowp float e3 = 1.0 - linearstep(1.0, 1.0 + xPixel, tx.x);
                lowp float e4 = 1.0 - linearstep(1.0, 1.0 + yPixel, tx.y);

                lowp vec4 sample = texture2D(source, tx);
                sample.rgb *= e1 * e2 * e3 * e4;
                gl_FragColor = sample * qt_Opacity * offset.a;
            }
        "
    }
}
