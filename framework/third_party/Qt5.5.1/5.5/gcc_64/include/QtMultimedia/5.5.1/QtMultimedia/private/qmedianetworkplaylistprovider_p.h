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

#ifndef QMEDIANETWORKPAYLISTPROVIDER_P_H
#define QMEDIANETWORKPAYLISTPROVIDER_P_H

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

#include "qmediaplaylistprovider_p.h"

QT_BEGIN_NAMESPACE


class QMediaNetworkPlaylistProviderPrivate;
class Q_MULTIMEDIA_EXPORT QMediaNetworkPlaylistProvider : public QMediaPlaylistProvider
{
    Q_OBJECT
public:
    QMediaNetworkPlaylistProvider(QObject *parent=0);
    virtual ~QMediaNetworkPlaylistProvider();

    virtual bool load(const QNetworkRequest &request, const char *format = 0);

    virtual int mediaCount() const;
    virtual QMediaContent media(int pos) const;

    virtual bool isReadOnly() const;

    virtual bool addMedia(const QMediaContent &content);
    virtual bool addMedia(const QList<QMediaContent> &items);
    virtual bool insertMedia(int pos, const QMediaContent &content);
    virtual bool insertMedia(int pos, const QList<QMediaContent> &items);
    virtual bool removeMedia(int pos);
    virtual bool removeMedia(int start, int end);
    virtual bool clear();

public Q_SLOTS:
    virtual void shuffle();

private:
    Q_DISABLE_COPY(QMediaNetworkPlaylistProvider)
    Q_DECLARE_PRIVATE(QMediaNetworkPlaylistProvider)
    Q_PRIVATE_SLOT(d_func(), void _q_handleParserError(QPlaylistFileParser::ParserError err, const QString &))
    Q_PRIVATE_SLOT(d_func(), void _q_handleNewItem(const QVariant& content))
};

QT_END_NAMESPACE


#endif // QMEDIANETWORKPAYLISTSOURCE_P_H
