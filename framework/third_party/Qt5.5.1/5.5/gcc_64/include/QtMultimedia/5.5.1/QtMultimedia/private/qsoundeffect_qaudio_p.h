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

#ifndef QSOUNDEFFECT_QAUDIO_H
#define QSOUNDEFFECT_QAUDIO_H

//
//  W A R N I N G
//  -------------
//
// This file is not part of the Qt API. It exists purely as an
// implementation detail. This header file may change from version to
// version without notice, or even be removed.
//
// We mean it.
//

#include <QtCore/qobject.h>
#include <QtCore/qurl.h>
#include "qaudiooutput.h"
#include "qsamplecache_p.h"
#include "qsoundeffect.h"

QT_BEGIN_NAMESPACE

class QSoundEffectPrivate;

class PrivateSoundSource : public QIODevice
{
    friend class QSoundEffectPrivate;
    Q_OBJECT
public:
    PrivateSoundSource(QSoundEffectPrivate* s);
    ~PrivateSoundSource() {}

    qint64 readData( char* data, qint64 len);
    qint64 writeData(const char* data, qint64 len);

private Q_SLOTS:
    void sampleReady();
    void decoderError();
    void stateChanged(QAudio::State);

private:
    QUrl           m_url;
    int            m_loopCount;
    int            m_runningCount;
    bool           m_playing;
    QSoundEffect::Status  m_status;
    QAudioOutput   *m_audioOutput;
    QSample        *m_sample;
    bool           m_muted;
    qreal          m_volume;
    bool           m_sampleReady;
    qint64         m_offset;
    QString        m_category;

    QSoundEffectPrivate *soundeffect;
};


class QSoundEffectPrivate : public QObject
{
    friend class PrivateSoundSource;
    Q_OBJECT
public:

    explicit QSoundEffectPrivate(QObject* parent);
    ~QSoundEffectPrivate();

    static QStringList supportedMimeTypes();

    QUrl source() const;
    void setSource(const QUrl &url);
    int loopCount() const;
    int loopsRemaining() const;
    void setLoopCount(int loopCount);
    qreal volume() const;
    void setVolume(qreal volume);
    bool isMuted() const;
    void setMuted(bool muted);
    bool isLoaded() const;
    bool isPlaying() const;
    QSoundEffect::Status status() const;

    void release();

    QString category() const;
    void setCategory(const QString &);

public Q_SLOTS:
    void play();
    void stop();

Q_SIGNALS:
    void loopsRemainingChanged();
    void volumeChanged();
    void mutedChanged();
    void loadedChanged();
    void playingChanged();
    void statusChanged();
    void categoryChanged();

private:
    void setStatus(QSoundEffect::Status status);
    void setPlaying(bool playing);
    void setLoopsRemaining(int loopsRemaining);

    PrivateSoundSource* d;
};

QT_END_NAMESPACE

#endif // QSOUNDEFFECT_QAUDIO_H
