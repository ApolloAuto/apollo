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
    property variant input
    property variant output
    property variant sourceRect
    visible: false

    Component.onCompleted: evaluateInput()

    onInputChanged: evaluateInput()

    onSourceRectChanged: evaluateInput()

    function evaluateInput() {
        if (input == undefined) {
            output =  input
        }
        else if (sourceRect != undefined && sourceRect != Qt.rect(0, 0, 0, 0) && !isQQuickShaderEffectSource(input)) {
            proxySource.sourceItem = input
            output = proxySource
            proxySource.sourceRect = sourceRect
        }
        else if (isQQuickItemLayerEnabled(input)) {
            output = input
        }
        else if ((isQQuickImage(input) && !hasTileMode(input) && !hasChildren(input))) {
            output = input
        }
        else if (isQQuickShaderEffectSource(input)) {
            output = input
        }
        else {
            proxySource.sourceItem = input
            output = proxySource
            proxySource.sourceRect = Qt.rect(0, 0, 0, 0)
        }
    }

    function isQQuickItemLayerEnabled(item) {
        if (item.hasOwnProperty("layer")) {
            var l = item["layer"]
            if (l.hasOwnProperty("enabled") && l["enabled"].toString() == "true")
                return true
        }
        return false
    }

    function isQQuickImage(item) {
        var imageProperties = [ "fillMode", "progress", "asynchronous", "sourceSize", "status", "smooth" ]
        return hasProperties(item, imageProperties)
    }

    function isQQuickShaderEffectSource(item) {
        var shaderEffectSourceProperties = [ "hideSource", "format", "sourceItem", "mipmap", "wrapMode", "live", "recursive", "sourceRect" ]
        return hasProperties(item, shaderEffectSourceProperties)
    }

    function hasProperties(item, properties) {
        var counter = 0
            for (var j = 0; j < properties.length; j++) {
                if (item.hasOwnProperty(properties [j]))
                    counter++
            }
        return properties.length == counter
    }

    function hasChildren(item) {
        if (item.hasOwnProperty("childrenRect")) {
            if (item["childrenRect"].toString() != "QRectF(0, 0, 0, 0)")
                return true
            else
                return false
        }
        return false
    }

    function hasTileMode(item) {
        if (item.hasOwnProperty("fillMode")) {
            if (item["fillMode"].toString() != "0")
                return true
            else
                return false
        }
        return false
    }

    ShaderEffectSource {
        id: proxySource
        live: rootItem.input != rootItem.output
        hideSource: false
        smooth: true
        visible: false
    }
}
