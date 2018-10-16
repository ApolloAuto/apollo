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

#ifndef QQUICKTRANSLATE_P_H
#define QQUICKTRANSLATE_P_H

#include "qquickitem.h"

#include <QtGui/qmatrix4x4.h>

QT_BEGIN_NAMESPACE

class QQuickTranslatePrivate;
class Q_AUTOTEST_EXPORT QQuickTranslate : public QQuickTransform
{
    Q_OBJECT

    Q_PROPERTY(qreal x READ x WRITE setX NOTIFY xChanged)
    Q_PROPERTY(qreal y READ y WRITE setY NOTIFY yChanged)

public:
    QQuickTranslate(QObject *parent = 0);
    ~QQuickTranslate();

    qreal x() const;
    void setX(qreal);

    qreal y() const;
    void setY(qreal);

    void applyTo(QMatrix4x4 *matrix) const Q_DECL_OVERRIDE;

Q_SIGNALS:
    void xChanged();
    void yChanged();

private:
    Q_DECLARE_PRIVATE(QQuickTranslate)
    Q_DISABLE_COPY(QQuickTranslate)
};

class QQuickScalePrivate;
class Q_AUTOTEST_EXPORT QQuickScale : public QQuickTransform
{
    Q_OBJECT

    Q_PROPERTY(QVector3D origin READ origin WRITE setOrigin NOTIFY originChanged)
    Q_PROPERTY(qreal xScale READ xScale WRITE setXScale NOTIFY xScaleChanged)
    Q_PROPERTY(qreal yScale READ yScale WRITE setYScale NOTIFY yScaleChanged)
    Q_PROPERTY(qreal zScale READ zScale WRITE setZScale NOTIFY zScaleChanged)
public:
    QQuickScale(QObject *parent = 0);
    ~QQuickScale();

    QVector3D origin() const;
    void setOrigin(const QVector3D &point);

    qreal xScale() const;
    void setXScale(qreal);

    qreal yScale() const;
    void setYScale(qreal);

    qreal zScale() const;
    void setZScale(qreal);

    void applyTo(QMatrix4x4 *matrix) const Q_DECL_OVERRIDE;

Q_SIGNALS:
    void originChanged();
    void xScaleChanged();
    void yScaleChanged();
    void zScaleChanged();
    void scaleChanged();

private:
    Q_DECLARE_PRIVATE(QQuickScale)
};

class QQuickRotationPrivate;
class Q_AUTOTEST_EXPORT QQuickRotation : public QQuickTransform
{
    Q_OBJECT

    Q_PROPERTY(QVector3D origin READ origin WRITE setOrigin NOTIFY originChanged)
    Q_PROPERTY(qreal angle READ angle WRITE setAngle NOTIFY angleChanged)
    Q_PROPERTY(QVector3D axis READ axis WRITE setAxis NOTIFY axisChanged)
public:
    QQuickRotation(QObject *parent = 0);
    ~QQuickRotation();

    QVector3D origin() const;
    void setOrigin(const QVector3D &point);

    qreal angle() const;
    void setAngle(qreal);

    QVector3D axis() const;
    void setAxis(const QVector3D &axis);
    void setAxis(Qt::Axis axis);

    void applyTo(QMatrix4x4 *matrix) const Q_DECL_OVERRIDE;

Q_SIGNALS:
    void originChanged();
    void angleChanged();
    void axisChanged();

private:
    Q_DECLARE_PRIVATE(QQuickRotation)
};

class QQuickMatrix4x4Private;
class Q_AUTOTEST_EXPORT QQuickMatrix4x4 : public QQuickTransform
{
    Q_OBJECT

    Q_PROPERTY(QMatrix4x4 matrix READ matrix WRITE setMatrix NOTIFY matrixChanged)
public:
    QQuickMatrix4x4(QObject *parent = 0);
    ~QQuickMatrix4x4();

    QMatrix4x4 matrix() const;
    void setMatrix(const QMatrix4x4& matrix);

    void applyTo(QMatrix4x4 *matrix) const Q_DECL_OVERRIDE;

Q_SIGNALS:
    void matrixChanged();

private:
    Q_DECLARE_PRIVATE(QQuickMatrix4x4)
};


QT_END_NAMESPACE

QML_DECLARE_TYPE(QQuickTranslate)

#endif
