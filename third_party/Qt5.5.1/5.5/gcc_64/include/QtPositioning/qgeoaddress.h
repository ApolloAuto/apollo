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

#ifndef QGEOADDRESS_H
#define QGEOADDRESS_H

#include <QtCore/QMetaType>
#include <QtCore/QSharedDataPointer>
#include <QtPositioning/qpositioningglobal.h>

QT_BEGIN_NAMESPACE

class QString;
class QGeoAddressPrivate;
class Q_POSITIONING_EXPORT QGeoAddress
{
public:
    QGeoAddress();
    QGeoAddress(const QGeoAddress &other);
    ~QGeoAddress();

    QGeoAddress &operator=(const QGeoAddress &other);
    bool operator==(const QGeoAddress &other) const;
    bool operator!=(const QGeoAddress &other) const {
        return !(other == *this);
    }

    QString text() const;
    void setText(const QString &text);

    QString country() const;
    void setCountry(const QString &country);

    QString countryCode() const;
    void setCountryCode(const QString &countryCode);

    QString state() const;
    void setState(const QString &state);

    QString county() const;
    void setCounty(const QString &county);

    QString city() const;
    void setCity(const QString &city);

    QString district() const;
    void setDistrict(const QString &district);

    QString postalCode() const;
    void setPostalCode(const QString &postalCode);

    QString street() const;
    void setStreet(const QString &street);

    bool isEmpty() const;
    void clear();

    bool isTextGenerated() const;

private:
    QSharedDataPointer<QGeoAddressPrivate> d;
};

Q_DECLARE_TYPEINFO(QGeoAddress, Q_MOVABLE_TYPE);

QT_END_NAMESPACE

Q_DECLARE_METATYPE(QGeoAddress)

#endif
