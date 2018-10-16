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

#ifndef MASKEXTRUDER_H
#define MASKEXTRUDER_H
#include "qquickparticleextruder_p.h"
#include <private/qquickpixmapcache_p.h>
#include <QUrl>
#include <QImage>

QT_BEGIN_NAMESPACE

class QQuickMaskExtruder : public QQuickParticleExtruder
{
    Q_OBJECT
    Q_PROPERTY(QUrl source READ source WRITE setSource NOTIFY sourceChanged)
public:
    explicit QQuickMaskExtruder(QObject *parent = 0);
    virtual QPointF extrude(const QRectF &);
    virtual bool contains(const QRectF &bounds, const QPointF &point);

    QUrl source() const
    {
        return m_source;
    }

Q_SIGNALS:

    void sourceChanged(QUrl arg);

public Q_SLOTS:
    void setSource(QUrl arg);

private Q_SLOTS:
    void startMaskLoading();
    void finishMaskLoading();

private:
    QUrl m_source;

    void ensureInitialized(const QRectF &r);
    int m_lastWidth;
    int m_lastHeight;
    QQuickPixmap m_pix;
    QImage m_img;
    QList<QPointF> m_mask;//TODO: More memory efficient datastructures
    //Perhaps just the mask for the largest bounds is stored, and interpolate up
};

QT_END_NAMESPACE

#endif // MASKEXTRUDER_H
