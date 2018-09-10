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

#ifndef QIMAGEENCODERCONTROL_H
#define QIMAGEENCODERCONTROL_H

#include <QtMultimedia/qmediacontrol.h>
#include <QtMultimedia/qmediarecorder.h>
#include <QtMultimedia/qmediaencodersettings.h>

#include <QtCore/qsize.h>

QT_BEGIN_NAMESPACE

class QByteArray;
class QStringList;
QT_END_NAMESPACE

QT_BEGIN_NAMESPACE

// Required for QDoc workaround
class QString;

class Q_MULTIMEDIA_EXPORT QImageEncoderControl : public QMediaControl
{
    Q_OBJECT

public:
    virtual ~QImageEncoderControl();

    virtual QStringList supportedImageCodecs() const = 0;
    virtual QString imageCodecDescription(const QString &codecName) const = 0;

    virtual QList<QSize> supportedResolutions(const QImageEncoderSettings &settings,
                                              bool *continuous = 0) const = 0;

    virtual QImageEncoderSettings imageSettings() const = 0;
    virtual void setImageSettings(const QImageEncoderSettings &settings) = 0;

protected:
    QImageEncoderControl(QObject *parent = 0);
};

#define QImageEncoderControl_iid "org.qt-project.qt.imageencodercontrol/5.0"
Q_MEDIA_DECLARE_CONTROL(QImageEncoderControl, QImageEncoderControl_iid)

QT_END_NAMESPACE


#endif
