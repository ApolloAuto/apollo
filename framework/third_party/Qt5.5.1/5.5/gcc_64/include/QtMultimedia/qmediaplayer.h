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

#ifndef QMEDIAPLAYER_H
#define QMEDIAPLAYER_H

#include <QtMultimedia/qmediaobject.h>
#include <QtMultimedia/qmediacontent.h>
#include <QtMultimedia/qmediaenumdebug.h>

#include <QtNetwork/qnetworkconfiguration.h>

QT_BEGIN_NAMESPACE


class QAbstractVideoSurface;
class QMediaPlaylist;
class QVideoWidget;
class QGraphicsVideoItem;

class QMediaPlayerPrivate;
class Q_MULTIMEDIA_EXPORT QMediaPlayer : public QMediaObject
{
    Q_OBJECT
    Q_PROPERTY(QMediaContent media READ media WRITE setMedia NOTIFY mediaChanged)
    Q_PROPERTY(QMediaContent currentMedia READ currentMedia NOTIFY currentMediaChanged)
    Q_PROPERTY(QMediaPlaylist * playlist READ playlist WRITE setPlaylist)
    Q_PROPERTY(qint64 duration READ duration NOTIFY durationChanged)
    Q_PROPERTY(qint64 position READ position WRITE setPosition NOTIFY positionChanged)
    Q_PROPERTY(int volume READ volume WRITE setVolume NOTIFY volumeChanged)
    Q_PROPERTY(bool muted READ isMuted WRITE setMuted NOTIFY mutedChanged)
    Q_PROPERTY(int bufferStatus READ bufferStatus NOTIFY bufferStatusChanged)
    Q_PROPERTY(bool audioAvailable READ isAudioAvailable NOTIFY audioAvailableChanged)
    Q_PROPERTY(bool videoAvailable READ isVideoAvailable NOTIFY videoAvailableChanged)
    Q_PROPERTY(bool seekable READ isSeekable NOTIFY seekableChanged)
    Q_PROPERTY(qreal playbackRate READ playbackRate WRITE setPlaybackRate NOTIFY playbackRateChanged)
    Q_PROPERTY(State state READ state NOTIFY stateChanged)
    Q_PROPERTY(MediaStatus mediaStatus READ mediaStatus NOTIFY mediaStatusChanged)
    Q_PROPERTY(QString error READ errorString)
    Q_ENUMS(State)
    Q_ENUMS(MediaStatus)
    Q_ENUMS(Error)

public:
    enum State
    {
        StoppedState,
        PlayingState,
        PausedState
    };

    enum MediaStatus
    {
        UnknownMediaStatus,
        NoMedia,
        LoadingMedia,
        LoadedMedia,
        StalledMedia,
        BufferingMedia,
        BufferedMedia,
        EndOfMedia,
        InvalidMedia
    };

    enum Flag
    {
        LowLatency = 0x01,
        StreamPlayback = 0x02,
        VideoSurface = 0x04
    };
    Q_DECLARE_FLAGS(Flags, Flag)

    enum Error
    {
        NoError,
        ResourceError,
        FormatError,
        NetworkError,
        AccessDeniedError,
        ServiceMissingError,
        MediaIsPlaylist
    };

    QMediaPlayer(QObject *parent = 0, Flags flags = 0);
    ~QMediaPlayer();

    static QMultimedia::SupportEstimate hasSupport(const QString &mimeType,
                                            const QStringList& codecs = QStringList(),
                                            Flags flags = 0);
    static QStringList supportedMimeTypes(Flags flags = 0);

    void setVideoOutput(QVideoWidget *);
    void setVideoOutput(QGraphicsVideoItem *);
    void setVideoOutput(QAbstractVideoSurface *surface);

    QMediaContent media() const;
    const QIODevice *mediaStream() const;
    QMediaPlaylist *playlist() const;
    QMediaContent currentMedia() const;

    State state() const;
    MediaStatus mediaStatus() const;

    qint64 duration() const;
    qint64 position() const;

    int volume() const;
    bool isMuted() const;
    bool isAudioAvailable() const;
    bool isVideoAvailable() const;

    int bufferStatus() const;

    bool isSeekable() const;
    qreal playbackRate() const;

    Error error() const;
    QString errorString() const;

    QNetworkConfiguration currentNetworkConfiguration() const;

    QMultimedia::AvailabilityStatus availability() const;

public Q_SLOTS:
    void play();
    void pause();
    void stop();

    void setPosition(qint64 position);
    void setVolume(int volume);
    void setMuted(bool muted);

    void setPlaybackRate(qreal rate);

    void setMedia(const QMediaContent &media, QIODevice *stream = 0);
    void setPlaylist(QMediaPlaylist *playlist);

    void setNetworkConfigurations(const QList<QNetworkConfiguration> &configurations);

Q_SIGNALS:
    void mediaChanged(const QMediaContent &media);
    void currentMediaChanged(const QMediaContent &media);

    void stateChanged(QMediaPlayer::State newState);
    void mediaStatusChanged(QMediaPlayer::MediaStatus status);

    void durationChanged(qint64 duration);
    void positionChanged(qint64 position);

    void volumeChanged(int volume);
    void mutedChanged(bool muted);
    void audioAvailableChanged(bool available);
    void videoAvailableChanged(bool videoAvailable);

    void bufferStatusChanged(int percentFilled);

    void seekableChanged(bool seekable);
    void playbackRateChanged(qreal rate);

    void error(QMediaPlayer::Error error);

    void networkConfigurationChanged(const QNetworkConfiguration &configuration);
public:
    virtual bool bind(QObject *);
    virtual void unbind(QObject *);

private:
    Q_DISABLE_COPY(QMediaPlayer)
    Q_DECLARE_PRIVATE(QMediaPlayer)
    Q_PRIVATE_SLOT(d_func(), void _q_stateChanged(QMediaPlayer::State))
    Q_PRIVATE_SLOT(d_func(), void _q_mediaStatusChanged(QMediaPlayer::MediaStatus))
    Q_PRIVATE_SLOT(d_func(), void _q_error(int, const QString &))
    Q_PRIVATE_SLOT(d_func(), void _q_updateMedia(const QMediaContent&))
    Q_PRIVATE_SLOT(d_func(), void _q_playlistDestroyed())
    Q_PRIVATE_SLOT(d_func(), void _q_handleMediaChanged(const QMediaContent&))
    Q_PRIVATE_SLOT(d_func(), void _q_handlePlaylistLoaded())
    Q_PRIVATE_SLOT(d_func(), void _q_handlePlaylistLoadFailed())
};

QT_END_NAMESPACE

Q_DECLARE_METATYPE(QMediaPlayer::State)
Q_DECLARE_METATYPE(QMediaPlayer::MediaStatus)
Q_DECLARE_METATYPE(QMediaPlayer::Error)

Q_MEDIA_ENUM_DEBUG(QMediaPlayer, State)
Q_MEDIA_ENUM_DEBUG(QMediaPlayer, MediaStatus)
Q_MEDIA_ENUM_DEBUG(QMediaPlayer, Error)

#endif  // QMEDIAPLAYER_H
