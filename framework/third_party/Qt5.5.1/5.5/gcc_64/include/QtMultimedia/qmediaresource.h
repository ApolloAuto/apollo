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

#ifndef QMEDIARESOURCE_H
#define QMEDIARESOURCE_H

#include <QtCore/qmap.h>
#include <QtCore/qmetatype.h>
#include <QtNetwork/qnetworkrequest.h>

#include <QtMultimedia/qtmultimediadefs.h>

QT_BEGIN_NAMESPACE

// Class forward declaration required for QDoc bug
class QString;
class Q_MULTIMEDIA_EXPORT QMediaResource
{
public:
    QMediaResource();
    QMediaResource(const QUrl &url, const QString &mimeType = QString());
    QMediaResource(const QNetworkRequest &request, const QString &mimeType = QString());
    QMediaResource(const QMediaResource &other);
    QMediaResource &operator =(const QMediaResource &other);
    ~QMediaResource();

    bool isNull() const;

    bool operator ==(const QMediaResource &other) const;
    bool operator !=(const QMediaResource &other) const;

    QUrl url() const;
    QNetworkRequest request() const;
    QString mimeType() const;

    QString language() const;
    void setLanguage(const QString &language);

    QString audioCodec() const;
    void setAudioCodec(const QString &codec);

    QString videoCodec() const;
    void setVideoCodec(const QString &codec);

    qint64 dataSize() const;
    void setDataSize(const qint64 size);

    int audioBitRate() const;
    void setAudioBitRate(int rate);

    int sampleRate() const;
    void setSampleRate(int frequency);

    int channelCount() const;
    void setChannelCount(int channels);

    int videoBitRate() const;
    void setVideoBitRate(int rate);

    QSize resolution() const;
    void setResolution(const QSize &resolution);
    void setResolution(int width, int height);


private:
    enum Property
    {
        Url,
        Request,
        MimeType,
        Language,
        AudioCodec,
        VideoCodec,
        DataSize,
        AudioBitRate,
        VideoBitRate,
        SampleRate,
        ChannelCount,
        Resolution
    };
    QMap<int, QVariant> values;
};

typedef QList<QMediaResource> QMediaResourceList;

QT_END_NAMESPACE

Q_DECLARE_METATYPE(QMediaResource)
Q_DECLARE_METATYPE(QMediaResourceList)

#endif
