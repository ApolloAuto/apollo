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

#ifndef QMEDIASERVICEPROVIDER_H
#define QMEDIASERVICEPROVIDER_H

#include <QtCore/qobject.h>
#include <QtCore/qshareddata.h>
#include <qtmultimediadefs.h>
#include "qmultimedia.h"
#include "qmediaserviceproviderplugin.h"

QT_BEGIN_NAMESPACE


class QMediaService;

class Q_MULTIMEDIA_EXPORT QMediaServiceProvider : public QObject
{
    Q_OBJECT

public:
    virtual QMediaService* requestService(const QByteArray &type, const QMediaServiceProviderHint &hint = QMediaServiceProviderHint()) = 0;
    virtual void releaseService(QMediaService *service) = 0;

    virtual QMediaServiceProviderHint::Features supportedFeatures(const QMediaService *service) const;

    virtual QMultimedia::SupportEstimate hasSupport(const QByteArray &serviceType,
                                             const QString &mimeType,
                                             const QStringList& codecs,
                                             int flags = 0) const;
    virtual QStringList supportedMimeTypes(const QByteArray &serviceType, int flags = 0) const;

    virtual QByteArray defaultDevice(const QByteArray &serviceType) const;
    virtual QList<QByteArray> devices(const QByteArray &serviceType) const;
    virtual QString deviceDescription(const QByteArray &serviceType, const QByteArray &device);

    virtual QCamera::Position cameraPosition(const QByteArray &device) const;
    virtual int cameraOrientation(const QByteArray &device) const;

    static QMediaServiceProvider* defaultServiceProvider();
    static void setDefaultServiceProvider(QMediaServiceProvider *provider);
};

QT_END_NAMESPACE


#endif  // QMEDIASERVICEPROVIDER_H
