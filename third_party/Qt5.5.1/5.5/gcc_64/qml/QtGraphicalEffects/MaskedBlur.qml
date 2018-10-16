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
    \qmltype MaskedBlur
    \inqmlmodule QtGraphicalEffects
    \since QtGraphicalEffects 1.0
    \inherits QtQuick2::Item
    \ingroup qtgraphicaleffects-blur
    \brief Applies a blur effect with a varying intesity.

    MaskedBlur effect softens the image by blurring it. The intensity of the
    blur can be controlled for each pixel using maskSource so that some parts of
    the source are blurred more than others. By default the effect produces a
    high quality result, thus the rendering speed may not be the highest
    possible. The rendering speed is reduced especially if the
    \l{MaskedBlur::samples}{samples} is large. For use cases that require faster
    rendering speed and the highest possible visual quality is not necessary,
    property \l{MaskedBlur::fast}{fast} can be set to true.

    \table
    \header
        \li Source
        \li MaskSource
        \li Effect applied
    \row
        \li \image Original_bug.png
        \li \image MaskedBlur_mask.png
        \li \image MaskedBlur_bug.png
    \endtable

    \section1 Example

    The following example shows how to apply the effect.
    \snippet MaskedBlur-example.qml example

*/
Item {
    id: rootItem

    /*!
        This property defines the source item that is going to be blurred.

        \note It is not supported to let the effect include itself, for
        instance by setting source to the effect's parent.
    */
    property variant source

    /*!
        This property defines the item that is controlling the final intensity
        of the blur. The pixel alpha channel value from maskSource defines the
        actual blur radius that is going to be used for blurring the
        corresponding source pixel.

        Opaque maskSource pixels produce blur with specified
        \l{MaskedBlur::radius}{radius}, while transparent pixels suppress the
        blur completely. Semitransparent maskSource pixels produce blur with a
        radius that is interpolated according to the pixel transparency level.
    */
    property variant maskSource

    /*!
        This property defines the distance of the neighboring pixels which
        affect the blurring of an individual pixel. A larger radius increases
        the blur effect.

        Depending on the radius value, value of the
        \l{MaskedBlur::samples}{samples} should be set to sufficiently large to
        ensure the visual quality.

        The value ranges from 0.0 (no blur) to inf. By default, the property is
        set to \c 0.0 (no blur).

        \table
        \header
        \li Output examples with different radius values
        \li
        \li
        \row
            \li \image MaskedBlur_radius1.png
            \li \image MaskedBlur_radius2.png
            \li \image MaskedBlur_radius3.png
        \row
            \li \b { radius: 0 }
            \li \b { radius: 8 }
            \li \b { radius: 16 }
        \row
            \li \l samples: 24
            \li \l samples: 24
            \li \l samples: 24
        \row
            \li \l transparentBorder: false
            \li \l transparentBorder: false
            \li \l transparentBorder: false
        \row
            \li \l fast: false
            \li \l fast: false
            \li \l fast: false
        \endtable

    */
    property real radius: 0.0

    /*!
        This property defines how many samples are taken per pixel when blur
        calculation is done. Larger value produces better quality, but is slower
        to render.

        Ideally, this value should be twice as large as the highest required
        radius value, for example, if the radius is animated between 0.0 and
        4.0, samples should be set to 8.

        The value ranges from 0 to 32. By default, the property is set to \c 0.

        This property is not intended to be animated. Changing this property may
        cause the underlying OpenGL shaders to be recompiled.

        When \l{MaskedBlur::fast}{fast} property is set to true, this property
        has no effect.
    */
    property int samples: 0

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

    /*!
        This property selects the blurring algorithm that is used to produce the
        blur. Setting this to true enables fast algorithm, setting value to
        false produces higher quality result.

        By default, the property is set to \c false.

        \table
        \header
        \li Output examples with different fast values
        \li
        \li
        \row
            \li \image MaskedBlur_fast1.png
            \li \image MaskedBlur_fast2.png
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
            \li \l transparentBorder: false
            \li \l transparentBorder: false
        \endtable

    */
    property bool fast: false

    /*!
        This property defines the blur behavior near the edges of the item,
        where the pixel blurring is affected by the pixels outside the source
        edges.

        If the property is set to \c true, the pixels outside the source are
        interpreted to be transparent, which is similar to OpenGL
        clamp-to-border extension. The blur is expanded slightly outside the
        effect item area.

        If the property is set to \c false, the pixels outside the source are
        interpreted to contain the same color as the pixels at the edge of the
        item, which is similar to OpenGL clamp-to-edge behavior. The blur does
        not expand outside the effect item area.

        By default, the property is set to \c false.

        \table
        \header
        \li Output examples with different transparentBorder values
        \li
        \li
        \row
            \li \image MaskedBlur_transparentBorder1.png
            \li \image MaskedBlur_transparentBorder2.png
        \row
            \li \b { transparentBorder: false }
            \li \b { transparentBorder: true }
        \row
            \li \l radius: 64
            \li \l radius: 64
        \row
            \li \l samples: 24
            \li \l samples: 24
        \row
            \li \l fast: true
            \li \l fast: true
        \endtable

    */
    property bool transparentBorder: false

    Loader {
        id: loaderItem
        anchors.fill: parent
        sourceComponent: rootItem.fast ? fastBlur : gaussianBlur
    }

    Component {
        id: gaussianBlur
        GaussianMaskedBlur {
            anchors.fill: parent
            source: rootItem.source
            maskSource: rootItem.maskSource
            radius: rootItem.radius
            maximumRadius: rootItem.samples * 0.5
            transparentBorder: rootItem.transparentBorder
            cached: rootItem.cached
        }
    }

    Component {
        id: fastBlur
        FastMaskedBlur {
            anchors.fill: parent
            source:rootItem. source
            maskSource: rootItem.maskSource
            blur: Math.pow(rootItem.radius / 64.0, 0.4)
            transparentBorder: rootItem.transparentBorder
            cached: rootItem.cached
        }
    }
}
