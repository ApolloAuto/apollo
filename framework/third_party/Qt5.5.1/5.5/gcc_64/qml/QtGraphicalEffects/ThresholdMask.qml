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
    \qmltype ThresholdMask
    \inqmlmodule QtGraphicalEffects
    \since QtGraphicalEffects 1.0
    \inherits QtQuick2::Item
    \ingroup qtgraphicaleffects-mask
    \brief Masks the source item with another item and applies a threshold
    value.

    The masking behavior can be controlled with the \l threshold value for the
    mask pixels.

    \table
    \header
        \li Source
        \li MaskSource
        \li Effect applied
    \row
        \li \image Original_bug.png
        \li \image ThresholdMask_mask.png
        \li \image ThresholdMask_bug.png
    \endtable

    \section1 Example

    The following example shows how to apply the effect.
    \snippet ThresholdMask-example.qml example
*/
Item {
    id: rootItem

    /*!
        This property defines the source item that is going to be masked.

        \note It is not supported to let the effect include itself, for
        instance by setting source to the effect's parent.
    */
    property variant source

    /*!
        This property defines the item that is going to be used as the mask.
        Mask item gets rendered into an intermediate pixel buffer and the alpha
        values from the result are used to determine the source item's pixels
        visibility in the display.

        \table
        \header
            \li Original
            \li Mask
            \li Effect applied
        \row
            \li \image Original_bug.png
            \li \image ThresholdMask_mask.png
            \li \image ThresholdMask_bug.png
        \endtable

        \note It is not supported to let the effect include itself, for
        instance by setting maskSource to the effect's parent.
    */
    property variant maskSource

    /*!
        This property defines a threshold value for the mask pixels. The mask
        pixels that have an alpha value below this property are used to
        completely mask away the corresponding pixels from the source item. The
        mask pixels that have a higher alpha value are used to alphablend the
        source item to the display.

        The value ranges from 0.0 (alpha value 0) to 1.0 (alpha value 255). By
        default, the property is set to \c 0.0.

        \table
        \header
        \li Output examples with different threshold values
        \li
        \li
        \row
            \li \image ThresholdMask_threshold1.png
            \li \image ThresholdMask_threshold2.png
            \li \image ThresholdMask_threshold3.png
        \row
            \li \b { threshold: 0.0 }
            \li \b { threshold: 0.5 }
            \li \b { threshold: 0.7 }
        \row
            \li \l spread: 0.2
            \li \l spread: 0.2
            \li \l spread: 0.2
        \endtable
    */
    property real threshold: 0.0

    /*!
        This property defines the smoothness of the mask edges near the
        \l{ThresholdMask::threshold}{threshold} alpha value. Setting spread to
        0.0 uses mask normally with the specified threshold. Setting higher
        spread values softens the transition from the transparent mask pixels
        towards opaque mask pixels by adding interpolated values between them.

        The value ranges from 0.0 (sharp mask edge) to 1.0 (smooth mask edge).
        By default, the property is set to \c 0.0.

        \table
        \header
        \li Output examples with different spread values
        \li
        \li
        \row
            \li \image ThresholdMask_spread1.png
            \li \image ThresholdMask_spread2.png
            \li \image ThresholdMask_spread3.png
        \row
            \li \b { spread: 0.0 }
            \li \b { spread: 0.2 }
            \li \b { spread: 0.8 }
        \row
            \li \l threshold: 0.4
            \li \l threshold: 0.4
            \li \l threshold: 0.4
        \endtable

    */
    property real spread: 0.0

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
        id: maskSourceProxy
        input: rootItem.maskSource
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
        property variant maskSource: maskSourceProxy.output
        property real threshold: rootItem.threshold
        property real spread: rootItem.spread

        anchors.fill: parent

        fragmentShader: "
            varying highp vec2 qt_TexCoord0;
            uniform highp float qt_Opacity;
            uniform lowp sampler2D source;
            uniform lowp sampler2D maskSource;
            uniform highp float threshold;
            uniform highp float spread;
            void main(void) {
                lowp vec4 colorFragment = texture2D(source, qt_TexCoord0.st);
                lowp vec4 maskFragment = texture2D(maskSource, qt_TexCoord0.st);
                gl_FragColor = colorFragment * smoothstep(threshold * (1.0 + spread) - spread, threshold * (1.0 + spread), maskFragment.a) * qt_Opacity;
            }
        "
    }
}
