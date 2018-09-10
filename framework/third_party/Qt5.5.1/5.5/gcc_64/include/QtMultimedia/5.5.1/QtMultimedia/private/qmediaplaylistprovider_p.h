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

#ifndef QMEDIAPLAYLISTPROVIDER_P_H
#define QMEDIAPLAYLISTPROVIDER_P_H

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

#include "qmediacontent.h"
#include "qmediaplaylist.h"

QT_BEGIN_NAMESPACE


class QMediaPlaylistProviderPrivate
{
public:
    QMediaPlaylistProviderPrivate()
    {}
    virtual ~QMediaPlaylistProviderPrivate()
    {}
};

class Q_MULTIMEDIA_EXPORT QMediaPlaylistProvider : public QObject
{
Q_OBJECT
public:
    QMediaPlaylistProvider(QObject *parent=0);
    virtual ~QMediaPlaylistProvider();

    virtual bool load(const QNetworkRequest &request, const char *format = 0);
    virtual bool load(QIODevice * device, const char *format = 0);
    virtual bool save(const QUrl &location, const char *format = 0);
    virtual bool save(QIODevice * device, const char *format);

    virtual int mediaCount() const = 0;
    virtual QMediaContent media(int index) const = 0;

    virtual bool isReadOnly() const;

    virtual bool addMedia(const QMediaContent &content);
    virtual bool addMedia(const QList<QMediaContent> &contentList);
    virtual bool insertMedia(int index, const QMediaContent &content);
    virtual bool insertMedia(int index, const QList<QMediaContent> &content);
    virtual bool removeMedia(int pos);
    virtual bool removeMedia(int start, int end);
    virtual bool clear();

public Q_SLOTS:
    virtual void shuffle();

Q_SIGNALS:
    void mediaAboutToBeInserted(int start, int end);
    void mediaInserted(int start, int end);

    void mediaAboutToBeRemoved(int start, int end);
    void mediaRemoved(int start, int end);

    void mediaChanged(int start, int end);

    void loaded();
    void loadFailed(QMediaPlaylist::Error, const QString& errorMessage);

protected:
    QMediaPlaylistProviderPrivate *d_ptr;
    QMediaPlaylistProvider(QMediaPlaylistProviderPrivate &dd, QObject *parent);

private:
    Q_DECLARE_PRIVATE(QMediaPlaylistProvider)
};

QT_END_NAMESPACE



#endif // QMEDIAPLAYLISTSOURCE_P_H
