/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the QtQuick module of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:LGPL21$
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and The Qt Company. For licensing terms
** and conditions see http://www.qt.io/terms-conditions. For further
** information use the contact form at http://www.qt.io/contact-us.
**
** GNU Lesser General Public License Usage
** Alternatively, this file may be used under the terms of the GNU Lesser
** General Public License version 2.1 or version 3 as published by the Free
** Software Foundation and appearing in the file LICENSE.LGPLv21 and
** LICENSE.LGPLv3 included in the packaging of this file. Please review the
** following information to ensure the GNU Lesser General Public License
** requirements will be met: https://www.gnu.org/licenses/lgpl.html and
** http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html.
**
** As a special exception, The Qt Company gives you certain additional
** rights. These rights are described in The Qt Company LGPL Exception
** version 1.1, included in the file LGPL_EXCEPTION.txt in this package.
**
** $QT_END_LICENSE$
**
****************************************************************************/

#ifndef QSGABSTRACTRENDERER_H
#define QSGABSTRACTRENDERER_H

#include <QtQuick/qsgnode.h>

QT_BEGIN_NAMESPACE

class QSGAbstractRendererPrivate;

class Q_QUICK_EXPORT QSGAbstractRenderer : public QObject
{
    Q_OBJECT
public:
    enum ClearModeBit
    {
        ClearColorBuffer    = 0x0001,
        ClearDepthBuffer    = 0x0002,
        ClearStencilBuffer  = 0x0004
    };
    Q_DECLARE_FLAGS(ClearMode, ClearModeBit)

    virtual ~QSGAbstractRenderer();

    void setRootNode(QSGRootNode *node);
    QSGRootNode *rootNode() const;
    void setDeviceRect(const QRect &rect);
    inline void setDeviceRect(const QSize &size) { setDeviceRect(QRect(QPoint(), size)); }
    QRect deviceRect() const;

    void setViewportRect(const QRect &rect);
    inline void setViewportRect(const QSize &size) { setViewportRect(QRect(QPoint(), size)); }
    QRect viewportRect() const;

    void setProjectionMatrixToRect(const QRectF &rect);
    void setProjectionMatrix(const QMatrix4x4 &matrix);
    QMatrix4x4 projectionMatrix() const;

    void setClearColor(const QColor &color);
    QColor clearColor() const;

    void setClearMode(ClearMode mode);
    ClearMode clearMode() const;

    virtual void renderScene(GLuint fboId = 0) = 0;

Q_SIGNALS:
    void sceneGraphChanged();

protected:
    QSGAbstractRenderer(QObject *parent = 0);
    virtual void nodeChanged(QSGNode *node, QSGNode::DirtyState state) = 0;

private:
    Q_DECLARE_PRIVATE(QSGAbstractRenderer)
    friend class QSGRootNode;
};

Q_DECLARE_OPERATORS_FOR_FLAGS(QSGAbstractRenderer::ClearMode)

QT_END_NAMESPACE

#endif
