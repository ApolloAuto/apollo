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
    \qmltype RadialGradient
    \inqmlmodule QtGraphicalEffects
    \since QtGraphicalEffects 1.0
    \inherits QtQuick2::Item
    \ingroup qtgraphicaleffects-gradient
    \brief Draws a radial gradient.

    A gradient is defined by two or more colors, which are blended seamlessly.
    The colors start from the middle of the item and end at the borders.

    \table
    \header
        \li Effect applied
    \row
        \li \image RadialGradient.png
    \endtable

    \section1 Example

    The following example shows how to apply the effect.
    \snippet RadialGradient-example.qml example

*/
Item {
    id: rootItem

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
        The HorizontalOffset and verticalOffset properties define the offset in
        pixels for the center point of the gradient compared to the item center.

        The values range from -inf to inf. By default, these properties are set
        to \c 0.

        \table
        \header
        \li Output examples with different horizontalOffset values
        \li
        \li
        \row
            \li \image RadialGradient_horizontalOffset1.png
            \li \image RadialGradient_horizontalOffset2.png
            \li \image RadialGradient_horizontalOffset3.png
        \row
            \li \b { horizontalOffset: -150 }
            \li \b { horizontalOffset: 0 }
            \li \b { horizontalOffset: 150 }
        \row
            \li \l verticalOffset: 0
            \li \l verticalOffset: 0
            \li \l verticalOffset: 0
        \row
            \li \l horizontalRadius: 300
            \li \l horizontalRadius: 300
            \li \l horizontalRadius: 300
        \row
            \li \l verticalRadius: 300
            \li \l verticalRadius: 300
            \li \l verticalRadius: 300
        \row
            \li \l angle: 0
            \li \l angle: 0
            \li \l angle: 0
        \endtable

    */
    property real horizontalOffset: 0.0
    property real verticalOffset: 0.0

    /*!
        The HorizontalRadius and verticalRadius properties define the shape and
        size of the radial gradient. If the radiuses are equal, the shape of the
        gradient is a circle. If the horizontal and vertical radiuses differ,
        the shape is elliptical. The radiuses are given in pixels.

        The value ranges from -inf to inf. By default, horizontalRadius is bound
        to width and verticalRadius is bound to height.

        \table
        \header
        \li Output examples with different horizontalRadius values
        \li
        \li
        \row
            \li \image RadialGradient_horizontalRadius1.png
            \li \image RadialGradient_horizontalRadius2.png
        \row
            \li \b { horizontalRadius: 300 }
            \li \b { horizontalRadius: 100 }
        \row
            \li \l horizontalOffset: 0
            \li \l horizontalOffset: 0
        \row
            \li \l verticalOffset: 0
            \li \l verticalOffset: 0
        \row
            \li \l verticalRadius: 300
            \li \l verticalRadius: 300
        \row
            \li \l angle: 0
            \li \l angle: 0
        \row
            \li \l gradient: QQuickGradient(0xa05fb10)
            \li \l gradient: QQuickGradient(0xa05fb10)
        \endtable

    */
    property real horizontalRadius: width
    property real verticalRadius: height

    /*!
        This property defines the rotation of the gradient around its center
        point. The rotation is only visible when the
        \l{RadialGradient::horizontalRadius}{horizontalRadius} and
        \l{RadialGradient::verticalRadius}{verticalRadius} properties are not
        equal. The angle is given in degrees and the default value is \c 0.

        \table
        \header
        \li Output examples with different angle values
        \li
        \li
        \row
            \li \image RadialGradient_angle1.png
            \li \image RadialGradient_angle2.png
            \li \image RadialGradient_angle3.png
        \row
            \li \b { angle: 0 }
            \li \b { angle: 45 }
            \li \b { angle: 90 }
        \row
            \li \l horizontalOffset: 0
            \li \l horizontalOffset: 0
            \li \l horizontalOffset: 0
        \row
            \li \l verticalOffset: 0
            \li \l verticalOffset: 0
            \li \l verticalOffset: 0
        \row
            \li \l horizontalRadius: 100
            \li \l horizontalRadius: 100
            \li \l horizontalRadius: 100
        \row
            \li \l verticalRadius: 300
            \li \l verticalRadius: 300
            \li \l verticalRadius: 300
        \endtable
    */
    property real angle: 0.0

    /*!
        This property defines the item that is going to be filled with gradient.
        Source item gets rendered into an intermediate pixel buffer and the
        alpha values from the result are used to determine the gradient's pixels
        visibility in the display. The default value for source is undefined and
        in that case whole effect area is filled with gradient.

        \table
        \header
        \li Output examples with different source values
        \li
        \li
        \row
            \li \image RadialGradient_maskSource1.png
            \li \image RadialGradient_maskSource2.png
        \row
            \li \b { source: undefined }
            \li \b { source: Image { source: images/butterfly.png } }
        \row
            \li \l horizontalOffset: 0
            \li \l horizontalOffset: 0
        \row
            \li \l verticalOffset: 0
            \li \l verticalOffset: 0
        \row
            \li \l horizontalRadius: 300
            \li \l horizontalRadius: 300
        \row
            \li \l verticalRadius: 300
            \li \l verticalRadius: 300
        \row
            \li \l angle: 0
            \li \l angle: 0
        \endtable

        \note It is not supported to let the effect include itself, for
        instance by setting source to the effect's parent.
    */
    property variant source

    /*!
        A gradient is defined by two or more colors, which are blended
        seamlessly. The colors are specified as a set of GradientStop child
        items, each of which defines a position on the gradient from 0.0 to 1.0
        and a color. The position of each GradientStop is defined by setting the
        position property. The color is defined by setting the color property.

        \table
        \header
        \li Output examples with different gradient values
        \li
        \li
        \row
            \li \image RadialGradient_gradient1.png
            \li \image RadialGradient_gradient2.png
            \li \image RadialGradient_gradient3.png
        \row
            \li \b {gradient:} \code
    Gradient {
      GradientStop {
       position: 0.000
       color: Qt.rgba(1, 0, 0, 1)
      }
      GradientStop {
       position: 0.167
       color: Qt.rgba(1, 1, 0, 1)
      }
      GradientStop {
       position: 0.333
       color: Qt.rgba(0, 1, 0, 1)
      }
      GradientStop {
       position: 0.500
       color: Qt.rgba(0, 1, 1, 1)
       }
      GradientStop {
       position: 0.667
       color: Qt.rgba(0, 0, 1, 1)
      }
      GradientStop {
       position: 0.833
       color: Qt.rgba(1, 0, 1, 1)
      }
      GradientStop {
       position: 1.000
       color: Qt.rgba(1, 0, 0, 1)
      }
    }
        \endcode
            \li \b {gradient:} \code
    Gradient {
      GradientStop {
        position: 0.0
        color: "#F0F0F0"
      }
      GradientStop {
        position: 0.5
        color: "#000000"
      }
      GradientStop {
        position: 1.0
        color: "#F0F0F0"
      }
    }
        \endcode
            \li \b {gradient:}
    \code
    Gradient {
      GradientStop {
        position: 0.0
        color: "#00000000"
      }
      GradientStop {
        position: 1.0
        color: "#FF000000"
      }
    }
    \endcode
        \row
            \li \l horizontalOffset: 0
            \li \l horizontalOffset: 0
            \li \l horizontalOffset: 0
        \row
            \li \l verticalOffset: 0
            \li \l verticalOffset: 0
            \li \l verticalOffset: 0
        \row
            \li \l horizontalRadius: 300
            \li \l horizontalRadius: 300
            \li \l horizontalRadius: 300
        \row
            \li \l verticalRadius: 300
            \li \l verticalRadius: 300
            \li \l verticalRadius: 300
        \row
            \li \l angle: 0
            \li \l angle: 0
            \li \l angle: 0
        \endtable
    */
    property Gradient gradient: Gradient {
        GradientStop { position: 0.0; color: "white" }
        GradientStop { position: 1.0; color: "black" }
    }

    SourceProxy {
        id: maskSourceProxy
        input: rootItem.source
    }

    ShaderEffectSource {
        id: gradientSource
        sourceItem: Rectangle {
            width: 16
            height: 256
            gradient: rootItem.gradient
            smooth: true
        }
        smooth: true
        hideSource: true
        visible: false
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
        property variant gradientImage: gradientSource
        property variant maskSource: maskSourceProxy.output
        property variant center: Qt.point(0.5 + rootItem.horizontalOffset / width, 0.5 + rootItem.verticalOffset / height)
        property real horizontalRatio: rootItem.horizontalRadius > 0 ? width / (2 * rootItem.horizontalRadius) : width * 16384
        property real verticalRatio: rootItem.verticalRadius > 0 ? height / (2 * rootItem.verticalRadius) : height * 16384
        property real angle: -rootItem.angle / 360 * 2 * Math.PI
        property variant matrixData: Qt.point(Math.sin(angle), Math.cos(angle))

        anchors.fill: parent

        vertexShader: "
            attribute highp vec4 qt_Vertex;
            attribute highp vec2 qt_MultiTexCoord0;
            uniform highp mat4 qt_Matrix;
            uniform highp vec2 matrixData;
            uniform highp float horizontalRatio;
            uniform highp float verticalRatio;
            uniform highp vec2 center;
            varying highp vec2 qt_TexCoord0;
            varying highp vec2 qt_TexCoord1;
            varying highp vec2 centerPoint;

            void main() {
                highp vec2 ratio = vec2(horizontalRatio, verticalRatio);

                // Rotation matrix
                highp mat2 rot = mat2(matrixData.y, -matrixData.x,
                                      matrixData.x,  matrixData.y);

                qt_TexCoord0 = qt_MultiTexCoord0;

                qt_TexCoord1 = qt_MultiTexCoord0;
                qt_TexCoord1 -= center;
                qt_TexCoord1 *= rot;
                qt_TexCoord1 += center;
                qt_TexCoord1 *= ratio;

                centerPoint = center * ratio;

                gl_Position = qt_Matrix * qt_Vertex;
            }
        "

        fragmentShader: maskSource == undefined ? noMaskShader : maskShader

        onFragmentShaderChanged: horizontalRatioChanged()

        property string maskShader: "
            uniform lowp sampler2D gradientImage;
            uniform lowp sampler2D maskSource;
            uniform lowp float qt_Opacity;
            varying highp vec2 qt_TexCoord0;
            varying highp vec2 qt_TexCoord1;
            varying highp vec2 centerPoint;

            void main() {
                lowp vec4 gradientColor = texture2D(gradientImage, vec2(0.0, 2.0 * distance(qt_TexCoord1, centerPoint)));
                lowp float maskAlpha = texture2D(maskSource, qt_TexCoord0).a;
                gl_FragColor = gradientColor * maskAlpha * qt_Opacity;
            }
        "

        property string noMaskShader: "
            uniform lowp sampler2D gradientImage;
            uniform lowp float qt_Opacity;
            varying highp vec2 qt_TexCoord1;
            varying highp vec2 centerPoint;

            void main() {
                lowp vec4 gradientColor = texture2D(gradientImage, vec2(0.0, 2.0 * distance(qt_TexCoord1, centerPoint)));
                gl_FragColor = gradientColor * qt_Opacity;
            }
        "
    }
}
