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
    \qmltype InnerShadow
    \inqmlmodule QtGraphicalEffects
    \since QtGraphicalEffects 1.0
    \inherits QtQuick2::Item
    \ingroup qtgraphicaleffects-drop-shadow
    \brief Generates a colorized and blurred shadow inside the
    source.

    By default the effect produces a high quality shadow image, thus the
    rendering speed of the shadow might not be the highest possible. The
    rendering speed is reduced especially if the shadow edges are heavily
    softened. For use cases that require faster rendering speed and for which
    the highest possible visual quality is not necessary, property
    \l{InnerShadow::fast}{fast} can be set to true.

    \table
    \header
        \li Source
        \li Effect applied
    \row
        \li \image Original_butterfly.png
        \li \image InnerShadow_butterfly.png
    \endtable

    \section1 Example

    The following example shows how to apply the effect.
    \snippet InnerShadow-example.qml example

*/
Item {
    id: rootItem

    /*!
        This property defines the source item that is going to be used as the
        source for the generated shadow.

        \note It is not supported to let the effect include itself, for
        instance by setting source to the effect's parent.
    */
    property variant source

    /*!
        Radius defines the softness of the shadow. A larger radius causes the
        edges of the shadow to appear more blurry.

        Depending on the radius value, value of the
        \l{InnerShadow::samples}{samples} should be set to sufficiently large to
        ensure the visual quality.

        The value ranges from 0.0 (no blur) to inf. By default, the property is
        set to \c 0.0 (no blur).

        \table
        \header
        \li Output examples with different radius values
        \li
        \li
        \row
            \li \image InnerShadow_radius1.png
            \li \image InnerShadow_radius2.png
            \li \image InnerShadow_radius3.png
        \row
            \li \b { radius: 0 }
            \li \b { radius: 6 }
            \li \b { radius: 12 }
        \row
            \li \l samples: 24
            \li \l samples: 24
            \li \l samples: 24
        \row
            \li \l color: #000000
            \li \l color: #000000
            \li \l color: #000000
        \row
            \li \l horizontalOffset: 0
            \li \l horizontalOffset: 0
            \li \l horizontalOffset: 0
        \row
            \li \l verticalOffset: 0
            \li \l verticalOffset: 0
            \li \l verticalOffset: 0
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

        When \l{InnerShadow::fast}{fast} property is set to true, this property
        has no effect.

    */
    property int samples: 0

    /*!
        This property defines how large part of the shadow color is strenghtened
        near the source edges.

        The value ranges from 0.0 to 1.0. By default, the property is set to \c
        0.5.

        \table
        \header
        \li Output examples with different spread values
        \li
        \li
        \row
            \li \image InnerShadow_spread1.png
            \li \image InnerShadow_spread2.png
            \li \image InnerShadow_spread3.png
        \row
            \li \b { spread: 0.0 }
            \li \b { spread: 0.3 }
            \li \b { spread: 0.5 }
        \row
            \li \l radius: 16
            \li \l radius: 16
            \li \l radius: 16
        \row
            \li \l samples: 24
            \li \l samples: 24
            \li \l samples: 24
        \row
            \li \l color: #000000
            \li \l color: #000000
            \li \l color: #000000
        \row
            \li \l horizontalOffset: 0
            \li \l horizontalOffset: 0
            \li \l horizontalOffset: 0
        \row
            \li \l verticalOffset: 0
            \li \l verticalOffset: 0
            \li \l verticalOffset: 0
        \endtable

    */
    property real spread: 0.0

    /*!
        This property defines the RGBA color value which is used for the shadow.

        By default, the property is set to \c "black".

        \table
        \header
        \li Output examples with different color values
        \li
        \li
        \row
            \li \image InnerShadow_color1.png
            \li \image InnerShadow_color2.png
            \li \image InnerShadow_color3.png
        \row
            \li \b { color: #000000 }
            \li \b { color: #ffffff }
            \li \b { color: #ff0000 }
        \row
            \li \l radius: 16
            \li \l radius: 16
            \li \l radius: 16
        \row
            \li \l samples: 24
            \li \l samples: 24
            \li \l samples: 24
        \row
            \li \l horizontalOffset: 0
            \li \l horizontalOffset: 0
            \li \l horizontalOffset: 0
        \row
            \li \l verticalOffset: 0
            \li \l verticalOffset: 0
            \li \l verticalOffset: 0
        \row
            \li \l spread: 0.2
            \li \l spread: 0.2
            \li \l spread: 0.2
        \endtable
    */
    property color color: "black"

    /*!
        \qmlproperty real QtGraphicalEffects1::InnerShadow::horizontalOffset
        \qmlproperty real QtGraphicalEffects1::InnerShadow::verticalOffset

        HorizontalOffset and verticalOffset properties define the offset for the
        rendered shadow compared to the InnerShadow item position. Often, the
        InnerShadow item is anchored so that it fills the source element. In
        this case, if the HorizontalOffset and verticalOffset properties are set
        to 0, the shadow is rendered fully inside the source item. By changing
        the offset properties, the shadow can be positioned relatively to the
        source item.

        The values range from -inf to inf. By default, the properties are set to
        \c 0.

        \table
        \header
        \li Output examples with different horizontalOffset values
        \li
        \li
        \row
            \li \image InnerShadow_horizontalOffset1.png
            \li \image InnerShadow_horizontalOffset2.png
            \li \image InnerShadow_horizontalOffset3.png
        \row
            \li \b { horizontalOffset: -20 }
            \li \b { horizontalOffset: 0 }
            \li \b { horizontalOffset: 20 }
        \row
            \li \l radius: 16
            \li \l radius: 16
            \li \l radius: 16
        \row
            \li \l samples: 24
            \li \l samples: 24
            \li \l samples: 24
        \row
            \li \l color: #000000
            \li \l color: #000000
            \li \l color: #000000
        \row
            \li \l verticalOffset: 0
            \li \l verticalOffset: 0
            \li \l verticalOffset: 0
        \row
            \li \l spread: 0
            \li \l spread: 0
            \li \l spread: 0
        \endtable

    */
    property real horizontalOffset: 0
    property real verticalOffset: 0

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
            \li \image InnerShadow_fast1.png
            \li \image InnerShadow_fast2.png
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
            \li \l color: #000000
            \li \l color: #000000
        \row
            \li \l horizontalOffset: 0
            \li \l horizontalOffset: 0
        \row
            \li \l verticalOffset: 0
            \li \l verticalOffset: 0
        \row
            \li \l spread: 0.2
            \li \l spread: 0.2
        \endtable
    */
    property bool fast: false

    /*!
        This property allows the effect output pixels to be cached in order to
        improve the rendering performance. Every time the source or effect
        properties are changed, the pixels in the cache must be updated. Memory
        consumption is increased, because an extra buffer of memory is required
        for storing the effect output.

        It is recommended to disable the cache when the source or the effect
        properties are animated.

        By default, the property is set to \c false.

    */
    property bool cached: false

    Loader {
        anchors.fill: parent
        sourceComponent: rootItem.fast ? innerShadow : gaussianInnerShadow
    }

    Component {
        id: gaussianInnerShadow
        GaussianInnerShadow {
            anchors.fill: parent
            source: rootItem.source
            radius: rootItem.radius
            maximumRadius: rootItem.samples * 0.5
            color: rootItem.color
            cached: rootItem.cached
            spread: rootItem.spread
            horizontalOffset: rootItem.horizontalOffset
            verticalOffset: rootItem.verticalOffset
        }
    }

    Component {
        id: innerShadow
        FastInnerShadow {
            anchors.fill: parent
            source: rootItem.source
            blur: Math.pow(rootItem.radius / 64.0, 0.4)
            color: rootItem.color
            cached: rootItem.cached
            spread: rootItem.spread
            horizontalOffset: rootItem.horizontalOffset
            verticalOffset: rootItem.verticalOffset
        }
    }
}
