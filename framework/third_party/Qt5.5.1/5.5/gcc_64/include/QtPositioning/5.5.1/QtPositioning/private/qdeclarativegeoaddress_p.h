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
***************************************************************************/

#ifndef QDECLARATIVEGEOADDRESS_P_H
#define QDECLARATIVEGEOADDRESS_P_H

#include <QtCore/QObject>
#include <QtPositioning/QGeoAddress>

QT_BEGIN_NAMESPACE

class Q_POSITIONING_EXPORT QDeclarativeGeoAddress : public QObject
{
    Q_OBJECT

    Q_PROPERTY(QGeoAddress address READ address WRITE setAddress)
    Q_PROPERTY(QString text READ text WRITE setText NOTIFY textChanged)
    Q_PROPERTY(QString country READ country WRITE setCountry NOTIFY countryChanged)
    Q_PROPERTY(QString countryCode READ countryCode WRITE setCountryCode NOTIFY countryCodeChanged)
    Q_PROPERTY(QString state READ state WRITE setState NOTIFY stateChanged)
    Q_PROPERTY(QString county READ county WRITE setCounty NOTIFY countyChanged)
    Q_PROPERTY(QString city READ city WRITE setCity NOTIFY cityChanged)
    Q_PROPERTY(QString district READ district WRITE setDistrict NOTIFY districtChanged)
    Q_PROPERTY(QString street READ street WRITE setStreet NOTIFY streetChanged)
    Q_PROPERTY(QString postalCode READ postalCode WRITE setPostalCode NOTIFY postalCodeChanged)
    Q_PROPERTY(bool isTextGenerated READ isTextGenerated NOTIFY isTextGeneratedChanged)

public:
    explicit QDeclarativeGeoAddress(QObject *parent = 0);
    QDeclarativeGeoAddress(const QGeoAddress &address, QObject *parent = 0);
    QGeoAddress address() const;
    void setAddress(const QGeoAddress &address);

    QString text() const;
    void setText(const QString &address);

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
    QString street() const;
    void setStreet(const QString &street);
    QString postalCode() const;
    void setPostalCode(const QString &postalCode);
    bool isTextGenerated() const;

Q_SIGNALS:
    void textChanged();
    void countryChanged();
    void countryCodeChanged();
    void stateChanged();
    void countyChanged();
    void cityChanged();
    void districtChanged();
    void streetChanged();
    void postalCodeChanged();
    void isTextGeneratedChanged();

private:
    QGeoAddress m_address;
};

QT_END_NAMESPACE

#endif // QDECLARATIVEGEOADDRESS_P_H
