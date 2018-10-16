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

#ifndef QMEDIAPLAYLIST_P_H
#define QMEDIAPLAYLIST_P_H

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

#include "qmediaplaylist.h"
#include "qmediaplaylistcontrol_p.h"
#include "qmediaplayer.h"
#include "qmediaplayercontrol.h"
#include "qmedianetworkplaylistprovider_p.h"
#include "qmediaobject_p.h"

#include <QtCore/qdebug.h>

#ifdef Q_MOC_RUN
# pragma Q_MOC_EXPAND_MACROS
#endif

QT_BEGIN_NAMESPACE


class QMediaPlaylistControl;
class QMediaPlaylistProvider;
class QMediaPlaylistReader;
class QMediaPlaylistWriter;
class QMediaPlayerControl;

class QMediaPlaylistPrivate
{
    Q_DECLARE_PUBLIC(QMediaPlaylist)
public:
    QMediaPlaylistPrivate()
        :mediaObject(0),
        control(0),
        networkPlaylistControl(0),
        error(QMediaPlaylist::NoError)
    {
    }

    virtual ~QMediaPlaylistPrivate() {}

    void _q_loadFailed(QMediaPlaylist::Error error, const QString &errorString)
    {
        this->error = error;
        this->errorString = errorString;

        emit q_ptr->loadFailed();
    }

    void _q_mediaObjectDeleted()
    {
        Q_Q(QMediaPlaylist);
        mediaObject = 0;
        if (control != networkPlaylistControl)
            control = 0;
        q->setMediaObject(0);
    }

    QMediaObject *mediaObject;

    QMediaPlaylistControl *control;
    QMediaPlaylistProvider *playlist() const { return control->playlistProvider(); }

    QMediaPlaylistControl *networkPlaylistControl;

    bool readItems(QMediaPlaylistReader *reader);
    bool writeItems(QMediaPlaylistWriter *writer);

    QMediaPlaylist::Error error;
    QString errorString;

    QMediaPlaylist *q_ptr;
};


class QMediaNetworkPlaylistControl : public QMediaPlaylistControl
{
    Q_OBJECT
public:
    QMediaNetworkPlaylistControl(QObject *parent)
        :QMediaPlaylistControl(parent)
    {
        QMediaPlaylistProvider *playlist = new QMediaNetworkPlaylistProvider(this);
        m_navigator = new QMediaPlaylistNavigator(playlist,this);
        m_navigator->setPlaybackMode(QMediaPlaylist::Sequential);

        connect(m_navigator, SIGNAL(currentIndexChanged(int)), SIGNAL(currentIndexChanged(int)));
        connect(m_navigator, SIGNAL(activated(QMediaContent)), SIGNAL(currentMediaChanged(QMediaContent)));
        connect(m_navigator, SIGNAL(playbackModeChanged(QMediaPlaylist::PlaybackMode)), SIGNAL(playbackModeChanged(QMediaPlaylist::PlaybackMode)));
    }

    virtual ~QMediaNetworkPlaylistControl() {};

    QMediaPlaylistProvider* playlistProvider() const { return m_navigator->playlist(); }
    bool setPlaylistProvider(QMediaPlaylistProvider *mediaPlaylist)
    {
        m_navigator->setPlaylist(mediaPlaylist);
        emit playlistProviderChanged();
        return true;
    }

    int currentIndex() const { return m_navigator->currentIndex(); }
    void setCurrentIndex(int position) { m_navigator->jump(position); }
    int nextIndex(int steps) const { return m_navigator->nextIndex(steps); }
    int previousIndex(int steps) const { return m_navigator->previousIndex(steps); }

    void next() { m_navigator->next(); }
    void previous() { m_navigator->previous(); }

    QMediaPlaylist::PlaybackMode playbackMode() const { return m_navigator->playbackMode(); }
    void setPlaybackMode(QMediaPlaylist::PlaybackMode mode) { m_navigator->setPlaybackMode(mode); }

private:
    QMediaPlaylistNavigator *m_navigator;
};


QT_END_NAMESPACE


#endif // QMEDIAPLAYLIST_P_H
