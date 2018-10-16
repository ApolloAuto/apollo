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

#ifndef QQUICKRECTANGLE_P_H
#define QQUICKRECTANGLE_P_H

#include "qquickitem.h"

#include <QtGui/qbrush.h>

#include <private/qtquickglobal_p.h>

QT_BEGIN_NAMESPACE

class Q_AUTOTEST_EXPORT QQuickPen : public QObject
{
    Q_OBJECT

    Q_PROPERTY(qreal width READ width WRITE setWidth NOTIFY penChanged)
    Q_PROPERTY(QColor color READ color WRITE setColor NOTIFY penChanged)
    Q_PROPERTY(bool pixelAligned READ pixelAligned WRITE setPixelAligned NOTIFY penChanged)
public:
    QQuickPen(QObject *parent=0);

    qreal width() const;
    void setWidth(qreal w);

    QColor color() const;
    void setColor(const QColor &c);

    bool pixelAligned() const;
    void setPixelAligned(bool aligned);

    bool isValid() const;

Q_SIGNALS:
    void penChanged();

private:
    qreal m_width;
    QColor m_color;
    bool m_aligned : 1;
    bool m_valid : 1;
};

class Q_QUICK_PRIVATE_EXPORT QQuickGradientStop : public QObject
{
    Q_OBJECT

    Q_PROPERTY(qreal position READ position WRITE setPosition)
    Q_PROPERTY(QColor color READ color WRITE setColor)

public:
    QQuickGradientStop(QObject *parent=0);

    qreal position() const;
    void setPosition(qreal position);

    QColor color() const;
    void setColor(const QColor &color);

private:
    void updateGradient();

private:
    qreal m_position;
    QColor m_color;
};

class Q_QUICK_PRIVATE_EXPORT QQuickGradient : public QObject
{
    Q_OBJECT

    Q_PROPERTY(QQmlListProperty<QQuickGradientStop> stops READ stops)
    Q_CLASSINFO("DefaultProperty", "stops")

public:
    QQuickGradient(QObject *parent=0);
    ~QQuickGradient();

    QQmlListProperty<QQuickGradientStop> stops();

    QGradientStops gradientStops() const;

Q_SIGNALS:
    void updated();

private:
    void doUpdate();

private:
    QList<QQuickGradientStop *> m_stops;
    friend class QQuickRectangle;
    friend class QQuickGradientStop;
};

class QQuickRectanglePrivate;
class Q_AUTOTEST_EXPORT QQuickRectangle : public QQuickItem
{
    Q_OBJECT

    Q_PROPERTY(QColor color READ color WRITE setColor NOTIFY colorChanged)
    Q_PROPERTY(QQuickGradient *gradient READ gradient WRITE setGradient RESET resetGradient)
    Q_PROPERTY(QQuickPen * border READ border CONSTANT)
    Q_PROPERTY(qreal radius READ radius WRITE setRadius NOTIFY radiusChanged)
public:
    QQuickRectangle(QQuickItem *parent=0);

    QColor color() const;
    void setColor(const QColor &);

    QQuickPen *border();

    QQuickGradient *gradient() const;
    void setGradient(QQuickGradient *gradient);
    void resetGradient();

    qreal radius() const;
    void setRadius(qreal radius);

Q_SIGNALS:
    void colorChanged();
    void radiusChanged();

protected:
    QSGNode *updatePaintNode(QSGNode *, UpdatePaintNodeData *) Q_DECL_OVERRIDE;

private Q_SLOTS:
    void doUpdate();

private:
    Q_DISABLE_COPY(QQuickRectangle)
    Q_DECLARE_PRIVATE(QQuickRectangle)
};

QT_END_NAMESPACE

QML_DECLARE_TYPE(QQuickPen)
QML_DECLARE_TYPE(QQuickGradientStop)
QML_DECLARE_TYPE(QQuickGradient)
QML_DECLARE_TYPE(QQuickRectangle)

#endif // QQUICKRECTANGLE_P_H
