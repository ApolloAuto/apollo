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

#ifndef QMEDIACONTENT_H
#define QMEDIACONTENT_H

#include <QtCore/qmetatype.h>
#include <QtCore/qshareddata.h>

#include <QtMultimedia/qmediaresource.h>

#include <QtMultimedia/qtmultimediadefs.h>

QT_BEGIN_NAMESPACE

class QMediaPlaylist;

class QMediaContentPrivate;
class Q_MULTIMEDIA_EXPORT QMediaContent
{
public:
    QMediaContent();
    QMediaContent(const QUrl &contentUrl);
    QMediaContent(const QNetworkRequest &contentRequest);
    QMediaContent(const QMediaResource &contentResource);
    QMediaContent(const QMediaResourceList &resources);
    QMediaContent(const QMediaContent &other);
    QMediaContent(QMediaPlaylist *playlist, const QUrl &contentUrl = QUrl(), bool takeOwnership = false);
    ~QMediaContent();

    QMediaContent& operator=(const QMediaContent &other);

    bool operator==(const QMediaContent &other) const;
    bool operator!=(const QMediaContent &other) const;

    bool isNull() const;

    QUrl canonicalUrl() const;
    QNetworkRequest canonicalRequest() const;
    QMediaResource canonicalResource() const;

    QMediaResourceList resources() const;

    QMediaPlaylist *playlist() const;
private:
    QSharedDataPointer<QMediaContentPrivate> d;
};

QT_END_NAMESPACE

Q_DECLARE_METATYPE(QMediaContent)

#endif  // QMEDIACONTENT_H
