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

#ifndef QSOUNDEFFECT_PULSE_H
#define QSOUNDEFFECT_PULSE_H

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


#include "qsoundeffect.h"

#include <QtCore/qobject.h>
#include <QtCore/qdatetime.h>
#include <qmediaplayer.h>
#include <pulse/pulseaudio.h>
#include "qsamplecache_p.h"

#include <private/qmediaresourcepolicy_p.h>
#include <private/qmediaresourceset_p.h>

QT_BEGIN_NAMESPACE

class QSoundEffectRef;

class QSoundEffectPrivate : public QObject
{
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
    void setCategory(const QString &category);

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

private Q_SLOTS:
    void decoderError();
    void sampleReady();
    void uploadSample();
    void contextReady();
    void contextFailed();
    void underRun();
    void prepare();
    void streamReady();
    void emptyComplete(void *stream);
    void updateVolume();
    void updateMuted();

    void handleAvailabilityChanged(bool available);

private:
    void playAvailable();
    void playSample();

    void emptyStream();
    void createPulseStream();
    void unloadPulseStream();

    void setPlaying(bool playing);
    void setStatus(QSoundEffect::Status status);
    void setLoopsRemaining(int loopsRemaining);

    static void stream_write_callback(pa_stream *s, size_t length, void *userdata);
    static void stream_state_callback(pa_stream *s, void *userdata);
    static void stream_underrun_callback(pa_stream *s, void *userdata);
    static void stream_cork_callback(pa_stream *s, int success, void *userdata);
    static void stream_flush_callback(pa_stream *s, int success, void *userdata);
    static void stream_write_done_callback(void *p);
    static void stream_adjust_prebuffer_callback(pa_stream *s, int success, void *userdata);
    static void stream_reset_buffer_callback(pa_stream *s, int success, void *userdata);
    static void setvolume_callback(pa_context *c, int success, void *userdata);
    static void setmuted_callback(pa_context *c, int success, void *userdata);

    pa_stream *m_pulseStream;
    int        m_sinkInputId;
    pa_sample_spec m_pulseSpec;
    int        m_pulseBufferSize;

    bool    m_emptying;
    bool    m_sampleReady;
    bool    m_playing;
    QSoundEffect::Status  m_status;
    bool    m_muted;
    bool    m_playQueued;
    bool    m_stopping;
    qreal     m_volume;
    int     m_loopCount;
    int     m_runningCount;
    QUrl    m_source;
    QByteArray m_name;
    QString m_category;
    bool m_reloadCategory;

    QSample *m_sample;
    int m_position;
    QSoundEffectRef *m_ref;

    bool m_resourcesAvailable;

    QMediaPlayerResourceSetInterface *m_resources;

#if defined(Q_WS_MAEMO_6) || defined(NEMO_AUDIO)
    bool m_customVolume;
#endif
};

QT_END_NAMESPACE

#endif // QSOUNDEFFECT_PULSE_H
