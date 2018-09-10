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
    property variant maskSource
    property real radius: 0.0
    property int maximumRadius: 0
    property bool cached: false
    property bool transparentBorder: false

    SourceProxy {
        id: sourceProxy
        input: rootItem.source
        sourceRect: rootItem.transparentBorder ? Qt.rect(-1, -1, parent.width + 2.0, parent.height + 2.0) : Qt.rect(0, 0, 0, 0)
    }

    SourceProxy {
        id: maskSourceProxy
        input: rootItem.maskSource
        sourceRect: rootItem.transparentBorder ? Qt.rect(-1, -1, parent.width + 2.0, parent.height + 2.0) : Qt.rect(0, 0, 0, 0)
    }

    ShaderEffectSource {
        id: cacheItem
        anchors.fill: blur
        visible: rootItem.cached
        smooth: true
        sourceItem: blur
        live: true
        hideSource: visible
    }

    GaussianDirectionalBlur {
        id: blur
        x: transparentBorder ? -maximumRadius - 1: 0
        y: transparentBorder ? -maximumRadius - 1: 0
        width: horizontalBlur.width
        height: horizontalBlur.height
        horizontalStep: 0.0
        verticalStep: 1.0 / parent.height
        source: horizontalBlur
        enableMask: true
        maskSource: maskSourceProxy.output
        radius: rootItem.radius
        maximumRadius: rootItem.maximumRadius
        transparentBorder: rootItem.transparentBorder
    }

    GaussianDirectionalBlur {
        id: horizontalBlur
        width: transparentBorder ? parent.width + 2 * maximumRadius + 2 : parent.width
        height: transparentBorder ? parent.height + 2 * maximumRadius + 2  : parent.height
        horizontalStep: 1.0 / parent.width
        verticalStep: 0.0
        source: sourceProxy.output
        enableMask: true
        maskSource: maskSourceProxy.output
        radius: rootItem.radius
        maximumRadius: rootItem.maximumRadius
        transparentBorder: rootItem.transparentBorder
        visible: false
    }
}
