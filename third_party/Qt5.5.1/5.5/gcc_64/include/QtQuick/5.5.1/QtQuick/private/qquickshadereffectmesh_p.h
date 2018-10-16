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

#include "qqmlparserstatus.h"

#include <QtQuick/qtquickglobal.h>
#include <QtGui/qcolor.h>
#include <QtCore/qobject.h>
#include <QtCore/qsize.h>
#include <QtCore/qvariant.h>
#include <QtGui/qopenglfunctions.h>

#ifndef QQUICKSHADEREFFECTMESH_P_H
#define QQUICKSHADEREFFECTMESH_P_H

QT_BEGIN_NAMESPACE

class QSGGeometry;
class QRectF;

class QQuickShaderEffectMesh : public QObject
{
    Q_OBJECT
public:
    QQuickShaderEffectMesh(QObject *parent = 0);
    // If 'geometry' != 0, 'attributes' is the same as last time the function was called.
    virtual QSGGeometry *updateGeometry(QSGGeometry *geometry, const QVector<QByteArray> &attributes, const QRectF &srcRect, const QRectF &rect) = 0;
    // If updateGeometry() fails, the reason should appear in the log.
    virtual QString log() const { return QString(); }

Q_SIGNALS:
    // Emitted when the geometry needs to be updated.
    void geometryChanged();
};

class QQuickGridMesh : public QQuickShaderEffectMesh
{
    Q_OBJECT
    Q_PROPERTY(QSize resolution READ resolution WRITE setResolution NOTIFY resolutionChanged)
public:
    QQuickGridMesh(QObject *parent = 0);
    QSGGeometry *updateGeometry(QSGGeometry *geometry, const QVector<QByteArray> &attributes, const QRectF &srcRect, const QRectF &rect) Q_DECL_OVERRIDE;
    QString log() const  Q_DECL_OVERRIDE { return m_log; }

    void setResolution(const QSize &res);
    QSize resolution() const;

Q_SIGNALS:
    void resolutionChanged();

private:
    QSize m_resolution;
    QString m_log;
};

inline QColor qt_premultiply_color(const QColor &c)
{
    return QColor::fromRgbF(c.redF() * c.alphaF(), c.greenF() * c.alphaF(), c.blueF() * c.alphaF(), c.alphaF());
}


QT_END_NAMESPACE

#endif // QQUICKSHADEREFFECTMESH_P_H
