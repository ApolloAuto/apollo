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

#ifndef QQUICKSPRITE_P_H
#define QQUICKSPRITE_P_H

#include <QObject>
#include <QUrl>
#include <QVariantMap>
#include <QQmlListProperty>
#include <QtQuick/private/qquickpixmapcache_p.h>
#include "qquickspriteengine_p.h"
#include <QDebug>

QT_BEGIN_NAMESPACE

class QQuickSprite : public QQuickStochasticState
{
    Q_OBJECT
    Q_PROPERTY(QUrl source READ source WRITE setSource NOTIFY sourceChanged)
    //Renderers have to query this hint when advancing frames
    Q_PROPERTY(bool reverse READ reverse WRITE setReverse NOTIFY reverseChanged)
    Q_PROPERTY(bool frameSync READ frameSync WRITE setFrameSync NOTIFY frameSyncChanged)
    Q_PROPERTY(int frames READ frames WRITE setFrames NOTIFY frameCountChanged)
    Q_PROPERTY(int frameCount READ frameCount WRITE setFrameCount NOTIFY frameCountChanged)
    //If frame height or width is not specified, it is assumed to be a single long row of square frames.
    //Otherwise, it can be multiple contiguous rows, when one row runs out the next will be used.
    Q_PROPERTY(int frameHeight READ frameHeight WRITE setFrameHeight NOTIFY frameHeightChanged)
    Q_PROPERTY(int frameWidth READ frameWidth WRITE setFrameWidth NOTIFY frameWidthChanged)
    Q_PROPERTY(int frameX READ frameX WRITE setFrameX NOTIFY frameXChanged)
    Q_PROPERTY(int frameY READ frameY WRITE setFrameY NOTIFY frameYChanged)
    //Precedence order: frameRate, frameDuration, duration
    Q_PROPERTY(qreal frameRate READ frameRate WRITE setFrameRate NOTIFY frameRateChanged RESET resetFrameRate)
    Q_PROPERTY(qreal frameRateVariation READ frameRateVariation WRITE setFrameRateVariation NOTIFY frameRateVariationChanged)
    Q_PROPERTY(int frameDuration READ frameDuration WRITE setFrameDuration NOTIFY frameDurationChanged RESET resetFrameDuration)
    Q_PROPERTY(int frameDurationVariation READ frameDurationVariation WRITE setFrameDurationVariation NOTIFY frameDurationVariationChanged)

public:
    explicit QQuickSprite(QObject *parent = 0);

    QUrl source() const
    {
        return m_source;
    }

    int frameHeight() const
    {
        return m_frameHeight;
    }

    int frameWidth() const
    {
        return m_frameWidth;
    }

    bool reverse() const
    {
        return m_reverse;
    }

    int frames() const
    {
        return m_frames;
    }

    int frameCount() const
    {
        return m_frames;
    }

    int frameX() const
    {
        return m_frameX;
    }

    int frameY() const
    {
        return m_frameY;
    }

    void resetFrameRate()
    {
        setFrameRate(-1);
    }

    qreal frameRate() const
    {
        return m_frameRate;
    }

    qreal frameRateVariation() const
    {
        return m_frameRateVariation;
    }

    void resetFrameDuration()
    {
        setFrameDuration(-1);
    }

    int frameDuration() const
    {
        return m_frameDuration;
    }

    int frameDurationVariation() const
    {
        return m_frameDurationVariation;
    }

    int variedDuration() const Q_DECL_OVERRIDE;

    bool frameSync() const
    {
        return m_frameSync;
    }

Q_SIGNALS:

    void sourceChanged(QUrl arg);

    void frameHeightChanged(int arg);

    void frameWidthChanged(int arg);

    void reverseChanged(bool arg);

    void frameCountChanged(int arg);

    void frameXChanged(int arg);

    void frameYChanged(int arg);

    void frameRateChanged(qreal arg);

    void frameRateVariationChanged(qreal arg);

    void frameDurationChanged(int arg);

    void frameDurationVariationChanged(int arg);

    void frameSyncChanged(bool arg);

public Q_SLOTS:

    void setSource(QUrl arg)
    {
        if (m_source != arg) {
            m_source = arg;
            Q_EMIT sourceChanged(arg);
            startImageLoading();
        }
    }

    void setFrameHeight(int arg)
    {
        if (m_frameHeight != arg) {
            m_frameHeight = arg;
            Q_EMIT frameHeightChanged(arg);
        }
    }

    void setFrameWidth(int arg)
    {
        if (m_frameWidth != arg) {
            m_frameWidth = arg;
            Q_EMIT frameWidthChanged(arg);
        }
    }

    void setReverse(bool arg)
    {
        if (m_reverse != arg) {
            m_reverse = arg;
            Q_EMIT reverseChanged(arg);
        }
    }

    void setFrames(int arg)
    {
        qWarning() << "Sprite::frames has been renamed Sprite::frameCount";
        setFrameCount(arg);
    }

    void setFrameCount(int arg)
    {
        if (m_frames != arg) {
            m_frames = arg;
            Q_EMIT frameCountChanged(arg);
        }
    }

    void setFrameX(int arg)
    {
        if (m_frameX != arg) {
            m_frameX = arg;
            Q_EMIT frameXChanged(arg);
        }
    }

    void setFrameY(int arg)
    {
        if (m_frameY != arg) {
            m_frameY = arg;
            Q_EMIT frameYChanged(arg);
        }
    }

    void setFrameRate(qreal arg)
    {
        if (m_frameRate != arg) {
            m_frameRate = arg;
            Q_EMIT frameRateChanged(arg);
        }
    }

    void setFrameRateVariation(qreal arg)
    {
        if (m_frameRateVariation != arg) {
            m_frameRateVariation = arg;
            Q_EMIT frameRateVariationChanged(arg);
        }
    }

    void setFrameDuration(int arg)
    {
        if (m_frameDuration != arg) {
            m_frameDuration = arg;
            Q_EMIT frameDurationChanged(arg);
        }
    }

    void setFrameDurationVariation(int arg)
    {
        if (m_frameDurationVariation != arg) {
            m_frameDurationVariation = arg;
            Q_EMIT frameDurationVariationChanged(arg);
        }
    }

    void setFrameSync(bool arg)
    {
        if (m_frameSync != arg) {
            m_frameSync = arg;
            Q_EMIT frameSyncChanged(arg);
        }
    }

private Q_SLOTS:
    void startImageLoading();

private:
    friend class QQuickImageParticle;
    friend class QQuickSpriteSequence;
    friend class QQuickAnimatedSprite;
    friend class QQuickSpriteEngine;
    friend class QQuickStochasticEngine;
    int m_generatedCount;
    int m_framesPerRow;
    int m_rowY;
    int m_rowStartX;

    QUrl m_source;
    bool m_reverse;
    int m_frameHeight;
    int m_frameWidth;
    int m_frames;
    int m_frameX;
    int m_frameY;
    qreal m_frameRate;
    qreal m_frameRateVariation;
    int m_frameDuration;
    int m_frameDurationVariation;
    bool m_frameSync;
    QQuickPixmap m_pix;
};

QT_END_NAMESPACE
#endif // QQUICKSPRITE_P_H
