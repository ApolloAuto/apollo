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

#ifndef QMEDIAENCODERSETTINGS_H
#define QMEDIAENCODERSETTINGS_H

#include <QtCore/qsharedpointer.h>
#include <QtCore/qstring.h>
#include <QtCore/qsize.h>
#include <QtCore/qvariant.h>
#include <QtMultimedia/qtmultimediadefs.h>
#include <QtMultimedia/qmultimedia.h>

QT_BEGIN_NAMESPACE



class QAudioEncoderSettingsPrivate;
class Q_MULTIMEDIA_EXPORT QAudioEncoderSettings
{
public:
    QAudioEncoderSettings();
    QAudioEncoderSettings(const QAudioEncoderSettings& other);

    ~QAudioEncoderSettings();

    QAudioEncoderSettings& operator=(const QAudioEncoderSettings &other);
    bool operator==(const QAudioEncoderSettings &other) const;
    bool operator!=(const QAudioEncoderSettings &other) const;

    bool isNull() const;

    QMultimedia::EncodingMode encodingMode() const;
    void setEncodingMode(QMultimedia::EncodingMode);

    QString codec() const;
    void setCodec(const QString& codec);

    int bitRate() const;
    void setBitRate(int bitrate);

    int channelCount() const;
    void setChannelCount(int channels);

    int sampleRate() const;
    void setSampleRate(int rate);

    QMultimedia::EncodingQuality quality() const;
    void setQuality(QMultimedia::EncodingQuality quality);

    QVariant encodingOption(const QString &option) const;
    QVariantMap encodingOptions() const;
    void setEncodingOption(const QString &option, const QVariant &value);
    void setEncodingOptions(const QVariantMap &options);

private:
    QSharedDataPointer<QAudioEncoderSettingsPrivate> d;
};

class QVideoEncoderSettingsPrivate;
class Q_MULTIMEDIA_EXPORT QVideoEncoderSettings
{
public:
    QVideoEncoderSettings();
    QVideoEncoderSettings(const QVideoEncoderSettings& other);

    ~QVideoEncoderSettings();

    QVideoEncoderSettings& operator=(const QVideoEncoderSettings &other);
    bool operator==(const QVideoEncoderSettings &other) const;
    bool operator!=(const QVideoEncoderSettings &other) const;

    bool isNull() const;

    QMultimedia::EncodingMode encodingMode() const;
    void setEncodingMode(QMultimedia::EncodingMode);

    QString codec() const;
    void setCodec(const QString &);

    QSize resolution() const;
    void setResolution(const QSize &);
    void setResolution(int width, int height);

    qreal frameRate() const;
    void setFrameRate(qreal rate);

    int bitRate() const;
    void setBitRate(int bitrate);

    QMultimedia::EncodingQuality quality() const;
    void setQuality(QMultimedia::EncodingQuality quality);

    QVariant encodingOption(const QString &option) const;
    QVariantMap encodingOptions() const;
    void setEncodingOption(const QString &option, const QVariant &value);
    void setEncodingOptions(const QVariantMap &options);

private:
    QSharedDataPointer<QVideoEncoderSettingsPrivate> d;
};

class QImageEncoderSettingsPrivate;
class Q_MULTIMEDIA_EXPORT QImageEncoderSettings
{
public:
    QImageEncoderSettings();
    QImageEncoderSettings(const QImageEncoderSettings& other);

    ~QImageEncoderSettings();

    QImageEncoderSettings& operator=(const QImageEncoderSettings &other);
    bool operator==(const QImageEncoderSettings &other) const;
    bool operator!=(const QImageEncoderSettings &other) const;

    bool isNull() const;

    QString codec() const;
    void setCodec(const QString &);

    QSize resolution() const;
    void setResolution(const QSize &);
    void setResolution(int width, int height);

    QMultimedia::EncodingQuality quality() const;
    void setQuality(QMultimedia::EncodingQuality quality);

    QVariant encodingOption(const QString &option) const;
    QVariantMap encodingOptions() const;
    void setEncodingOption(const QString &option, const QVariant &value);
    void setEncodingOptions(const QVariantMap &options);

private:
    QSharedDataPointer<QImageEncoderSettingsPrivate> d;
};

QT_END_NAMESPACE

Q_DECLARE_METATYPE(QAudioEncoderSettings)
Q_DECLARE_METATYPE(QVideoEncoderSettings)
Q_DECLARE_METATYPE(QImageEncoderSettings)


#endif
