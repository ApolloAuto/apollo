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
    \qmltype Glow
    \inqmlmodule QtGraphicalEffects
    \since QtGraphicalEffects 1.0
    \inherits QtQuick2::Item
    \ingroup qtgraphicaleffects-glow
    \brief Generates a blurred and colorized image of the source and places it
    behind the original, giving impression that the source is glowing.

    By default effect produces a high quality glow image, thus the rendering
    speed of the effect may not be the highest possible. The rendering speed is
    reduced especially if the glow edges are heavily softened. For use cases
    that require faster rendering speed and the highest possible visual quality
    is not necessary, property \l{Glow::fast}{fast} can be set to true.

    \table
    \header
        \li Source
        \li Effect applied
    \row
        \li \image Original_butterfly_black.png
        \li \image Glow_butterfly.png
    \endtable

    \section1 Example

    The following example shows how to apply the effect.
    \snippet Glow-example.qml example

*/
Item {
    id: rootItem

    /*!
        This property defines the source item that is going to be used as source
        for the generated glow.

        \note It is not supported to let the effect include itself, for
        instance by setting source to the effect's parent.
    */
    property variant source

    /*!
        Radius defines the softness of the glow. A larger radius causes the
        edges of the glow to appear more blurry.

        Depending on the radius value, value of the \l{Glow::samples}{samples}
        should be set to sufficiently large to ensure the visual quality.

        The value ranges from 0.0 (no blur) to inf. By default, the property is
        set to \c 0.0 (no blur).

        \table
        \header
        \li Output examples with different radius values
        \li
        \li
        \row
            \li \image Glow_radius1.png
            \li \image Glow_radius2.png
            \li \image Glow_radius3.png
        \row
            \li \b { radius: 0 }
            \li \b { radius: 6 }
            \li \b { radius: 12 }
        \row
            \li \l samples: 24
            \li \l samples: 24
            \li \l samples: 24
        \row
            \li \l color: #ffffff
            \li \l color: #ffffff
            \li \l color: #ffffff
        \row
            \li \l spread: 0
            \li \l spread: 0
            \li \l spread: 0
        \endtable
    */
    property real radius: 0.0

    /*!
        This property defines how many samples are taken per pixel when edge
        softening blur calculation is done. Larger value produces better
        quality, but is slower to render.

        Ideally, this value should be twice as large as the highest required
        radius value, for example, if the radius is animated between 0.0 and
        4.0, samples should be set to 8.

        The value ranges from 0 to 32. By default, the property is set to \c 0.

        This property is not intended to be animated. Changing this property may
        cause the underlying OpenGL shaders to be recompiled.

        When \l fast property is set to true, this property has no effect.

    */
    property int samples: 0

    /*!
        This property defines how large part of the glow color is strenghtened
        near the source edges.

        The values range from 0.0 to 1.0. By default, the property is set to \c
        0.5.

        \table
        \header
        \li Output examples with different spread values
        \li
        \li
        \row
            \li \image Glow_spread1.png
            \li \image Glow_spread2.png
            \li \image Glow_spread3.png
        \row
            \li \b { spread: 0.0 }
            \li \b { spread: 0.5 }
            \li \b { spread: 1.0 }
        \row
            \li \l radius: 8
            \li \l radius: 8
            \li \l radius: 8
        \row
            \li \l samples: 16
            \li \l samples: 16
            \li \l samples: 16
        \row
            \li \l color: #ffffff
            \li \l color: #ffffff
            \li \l color: #ffffff
        \endtable
    */
    property real spread: 0.0

    /*!
        This property defines the RGBA color value which is used for the glow.

        By default, the property is set to \c "white".

        \table
        \header
        \li Output examples with different color values
        \li
        \li
        \row
            \li \image Glow_color1.png
            \li \image Glow_color2.png
            \li \image Glow_color3.png
        \row
            \li \b { color: #ffffff }
            \li \b { color: #00ff00 }
            \li \b { color: #aa00ff00 }
        \row
            \li \l radius: 8
            \li \l radius: 8
            \li \l radius: 8
        \row
            \li \l samples: 16
            \li \l samples: 16
            \li \l samples: 16
        \row
            \li \l spread: 0.5
            \li \l spread: 0.5
            \li \l spread: 0.5
        \endtable

    */
    property color color: "white"

    /*!
        This property selects the blurring algorithm that is used to produce the
        softness for the effect. Setting this to true enables fast algorithm,
        setting value to false produces higher quality result.

        By default, the property is set to \c false.

        \table
        \header
        \li Output examples with different fast values
        \li
        \li
        \row
            \li \image Glow_fast1.png
            \li \image Glow_fast2.png
        \row
            \li \b { fast: false }
            \li \b { fast: true }
        \row
            \li \l radius: 16
            \li \l radius: 16
        \row
            \li \l samples: 24
            \li \l samples: 24
        \row
            \li \l color: #ffffff
            \li \l color: #ffffff
        \row
            \li \l spread: 0.3
            \li \l spread: 0.3
        \endtable
    */
    property bool fast: false

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

    /*!
        This property determines whether or not the effect has a transparent
        border.

        When set to \c true, the exterior of the item is padded with a 1 pixel
        wide transparent edge, making sampling outside the source texture use
        transparency instead of the edge pixels. Without this property, an
        image which has opaque edges will not get a blurred edge.

        In the snippet below, the Rectangle on the left has transparent borders
        and has blurred edges, whereas the Rectangle on the right does not.

        \snippet Glow-transparentBorder-example.qml example

        \image transparentBorder.png
    */
    property bool transparentBorder: false

    Loader {
        anchors.fill: parent
        sourceComponent: rootItem.fast ? fastGlow : gaussianGlow
    }

    Component {
        id: gaussianGlow
        GaussianGlow {
            anchors.fill: parent
            source: sourceProxy.output
            radius: rootItem.radius
            maximumRadius: rootItem.samples * 0.5
            color: rootItem.color
            cached: rootItem.cached
            spread: rootItem.spread
            transparentBorder: rootItem.transparentBorder
        }
    }

    Component {
        id: fastGlow
        FastGlow {
            anchors.fill: parent
            source: sourceProxy.output
            blur: Math.pow(rootItem.radius / 64.0, 0.4)
            color: rootItem.color
            cached: rootItem.cached
            spread: rootItem.spread
            transparentBorder: rootItem.transparentBorder
        }
    }

    SourceProxy {
        id: sourceProxy
        input: rootItem.source
        sourceRect: rootItem.transparentBorder ? Qt.rect(-1, -1, parent.width + 2.0, parent.height + 2.0) : Qt.rect(0, 0, 0, 0)
    }
    ShaderEffect {
        anchors.fill: parent
        property variant source: sourceProxy.output
    }
}
