/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the QtEnginio module of the Qt Toolkit.
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

#ifndef ENGINIOOAUTH2AUTHENTICATION_H
#define ENGINIOOAUTH2AUTHENTICATION_H

#include <Enginio/enginioidentity.h>
#include <QtCore/qstring.h>

QT_BEGIN_NAMESPACE

class EnginioOAuth2AuthenticationPrivate;
class ENGINIOCLIENT_EXPORT EnginioOAuth2Authentication : public EnginioIdentity
{
    Q_OBJECT
    Q_PROPERTY(QString user READ user WRITE setUser NOTIFY userChanged)
    Q_PROPERTY(QString password READ password WRITE setPassword NOTIFY passwordChanged)
    Q_DECLARE_PRIVATE(EnginioOAuth2Authentication)

public:
    explicit EnginioOAuth2Authentication(QObject *parent = 0);
    ~EnginioOAuth2Authentication();

    QString user() const Q_REQUIRED_RESULT;
    QString password() const Q_REQUIRED_RESULT;

public Q_SLOTS:
    void setUser(const QString &user);
    void setPassword(const QString &password);

Q_SIGNALS:
    void userChanged(const QString &user);
    void passwordChanged(const QString &password);

protected:
    virtual void prepareSessionToken(EnginioClientConnectionPrivate *enginio) Q_DECL_OVERRIDE;
    virtual void removeSessionToken(EnginioClientConnectionPrivate *enginio) Q_DECL_OVERRIDE;
};

QT_END_NAMESPACE

#endif // ENGINIOOAUTH2AUTHENTICATION_H
