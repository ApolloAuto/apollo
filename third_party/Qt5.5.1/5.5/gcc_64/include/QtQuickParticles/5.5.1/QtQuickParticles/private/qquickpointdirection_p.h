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

#ifndef POINTVECTOR_H
#define POINTVECTOR_H
#include "qquickdirection_p.h"

QT_BEGIN_NAMESPACE

class QQuickPointDirection : public QQuickDirection
{
    Q_OBJECT
    Q_PROPERTY(qreal x READ x WRITE setX NOTIFY xChanged)
    Q_PROPERTY(qreal y READ y WRITE setY NOTIFY yChanged)
    Q_PROPERTY(qreal xVariation READ xVariation WRITE setXVariation NOTIFY xVariationChanged)
    Q_PROPERTY(qreal yVariation READ yVariation WRITE setYVariation NOTIFY yVariationChanged)
public:
    explicit QQuickPointDirection(QObject *parent = 0);
    virtual const QPointF sample(const QPointF &from);
    qreal x() const
    {
        return m_x;
    }

    qreal y() const
    {
        return m_y;
    }

    qreal xVariation() const
    {
        return m_xVariation;
    }

    qreal yVariation() const
    {
        return m_yVariation;
    }

Q_SIGNALS:

    void xChanged(qreal arg);

    void yChanged(qreal arg);

    void xVariationChanged(qreal arg);

    void yVariationChanged(qreal arg);

public Q_SLOTS:
    void setX(qreal arg)
    {
        if (m_x != arg) {
            m_x = arg;
            Q_EMIT xChanged(arg);
        }
    }

    void setY(qreal arg)
    {
        if (m_y != arg) {
            m_y = arg;
            Q_EMIT yChanged(arg);
        }
    }

    void setXVariation(qreal arg)
    {
        if (m_xVariation != arg) {
            m_xVariation = arg;
            Q_EMIT xVariationChanged(arg);
        }
    }

    void setYVariation(qreal arg)
    {
        if (m_yVariation != arg) {
            m_yVariation = arg;
            Q_EMIT yVariationChanged(arg);
        }
    }

private:

    qreal m_x;
    qreal m_y;
    qreal m_xVariation;
    qreal m_yVariation;
};

QT_END_NAMESPACE
#endif // POINTVECTOR_H
