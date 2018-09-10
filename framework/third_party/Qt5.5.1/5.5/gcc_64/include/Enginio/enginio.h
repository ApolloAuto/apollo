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

#ifndef ENGINIO_H
#define ENGINIO_H

#include <Enginio/enginioclient_global.h>
#include <QObject>

QT_BEGIN_NAMESPACE

#ifndef Q_QDOC
class ENGINIOCLIENT_EXPORT Enginio
{
    Q_GADGET
#else
namespace Enginio {
#endif
    Q_ENUMS(AuthenticationState)
    Q_ENUMS(Operation)
    Q_ENUMS(ErrorType)
    Q_ENUMS(Role)

#ifndef Q_QDOC
public:
#endif
    enum AuthenticationState {
        NotAuthenticated,
        Authenticating,
        Authenticated,
        AuthenticationFailure
    };

    enum Operation {
        ObjectOperation,
        AccessControlOperation,
        UserOperation,
        UsergroupOperation,
        UsergroupMembersOperation,
        FileOperation,

        // private
        SessionOperation,
        SearchOperation,
        FileChunkUploadOperation,
        FileGetDownloadUrlOperation
    };

    enum Role {
        InvalidRole = -1,
        SyncedRole = Qt::UserRole + 1,
        CreatedAtRole,
        UpdatedAtRole,
        IdRole,
        ObjectTypeRole,
        JsonObjectRole,
        CustomPropertyRole = Qt::UserRole + 10 // the first fully dynamic role
    };

    enum ErrorType {
        NoError,
        NetworkError,
        BackendError
    };
};

Q_DECLARE_TYPEINFO(Enginio::Operation, Q_PRIMITIVE_TYPE);
Q_DECLARE_TYPEINFO(Enginio::AuthenticationState, Q_PRIMITIVE_TYPE);
Q_DECLARE_TYPEINFO(Enginio::Role, Q_PRIMITIVE_TYPE);
Q_DECLARE_TYPEINFO(Enginio::ErrorType, Q_PRIMITIVE_TYPE);

QT_END_NAMESPACE

Q_DECLARE_METATYPE(Enginio::Operation)
Q_DECLARE_METATYPE(Enginio::AuthenticationState)
Q_DECLARE_METATYPE(Enginio::Role)
Q_DECLARE_METATYPE(Enginio::ErrorType)

#endif // ENGINIO_H
