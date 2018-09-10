/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the Qt Toolkit.
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
#ifndef QMEDIAGAPLESSPLAYBACKCONTROL_H
#define QMEDIAGAPLESSPLAYBACKCONTROL_H

#include <QtMultimedia/qmediacontrol.h>
#include <QtMultimedia/qmediacontent.h>

QT_BEGIN_NAMESPACE

// Required for QDoc workaround
class QString;

class Q_MULTIMEDIA_EXPORT QMediaGaplessPlaybackControl : public QMediaControl
{
    Q_OBJECT
public:
    virtual ~QMediaGaplessPlaybackControl();

    virtual QMediaContent nextMedia() const = 0;
    virtual void setNextMedia(const QMediaContent &media) = 0;

    virtual bool isCrossfadeSupported() const = 0;
    virtual qreal crossfadeTime() const = 0;
    virtual void setCrossfadeTime(qreal crossfadeTime) = 0;

Q_SIGNALS:
    void crossfadeTimeChanged(qreal crossfadeTime);
    void nextMediaChanged(const QMediaContent& media);
    void advancedToNextMedia();

protected:
    QMediaGaplessPlaybackControl(QObject* parent = 0);
};

#define QMediaGaplessPlaybackControl_iid "org.qt-project.qt.mediagaplessplaybackcontrol/5.0"
Q_MEDIA_DECLARE_CONTROL(QMediaGaplessPlaybackControl, QMediaGaplessPlaybackControl_iid)

QT_END_NAMESPACE

#endif // QMEDIAGAPLESSPLAYBACKCONTROL_H
