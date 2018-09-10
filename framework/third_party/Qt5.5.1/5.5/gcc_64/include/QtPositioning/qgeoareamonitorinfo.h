/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the QtPositioning module of the Qt Toolkit.
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

#ifndef QGEOAREAMONITORINFO_H
#define QGEOAREAMONITORINFO_H

#include <QtPositioning/QGeoCoordinate>
#include <QtPositioning/QGeoShape>
#include <QtCore/QSharedDataPointer>
#include <QtCore/QVariantMap>

QT_BEGIN_NAMESPACE

class QDataStream;
class QGeoAreaMonitorInfo;

#ifndef QT_NO_DATASTREAM
Q_POSITIONING_EXPORT QDataStream &operator<<(QDataStream &, const QGeoAreaMonitorInfo &);
Q_POSITIONING_EXPORT QDataStream &operator>>(QDataStream &, QGeoAreaMonitorInfo &);
#endif

class QGeoAreaMonitorInfoPrivate;
class Q_POSITIONING_EXPORT QGeoAreaMonitorInfo
{
public:
    explicit QGeoAreaMonitorInfo(const QString &name = QString());
    QGeoAreaMonitorInfo(const QGeoAreaMonitorInfo &other);
    ~QGeoAreaMonitorInfo();

    QGeoAreaMonitorInfo &operator=(const QGeoAreaMonitorInfo &other);

    bool operator==(const QGeoAreaMonitorInfo &other) const;
    bool operator!=(const QGeoAreaMonitorInfo &other) const;

    QString name() const;
    void setName(const QString &name);

    QString identifier() const;
    bool isValid() const;

    QGeoShape area() const;
    void setArea(const QGeoShape &newShape);

    QDateTime expiration() const;
    void setExpiration(const QDateTime &expiry);

    bool isPersistent() const;
    void setPersistent(bool isPersistent);

    QVariantMap notificationParameters() const;
    void setNotificationParameters(const QVariantMap &parameters);
private:
    QSharedDataPointer<QGeoAreaMonitorInfoPrivate> d;

#ifndef QT_NO_DATASTREAM
    friend Q_POSITIONING_EXPORT QDataStream &operator<<(QDataStream &, const QGeoAreaMonitorInfo &);
    friend Q_POSITIONING_EXPORT QDataStream &operator>>(QDataStream &, QGeoAreaMonitorInfo &);
#endif
};

#ifndef QT_NO_DEBUG_STREAM
Q_POSITIONING_EXPORT QDebug operator<<(QDebug, const QGeoAreaMonitorInfo &);
#endif

QT_END_NAMESPACE

#endif // QGEOAREAMONITORINFO_H
