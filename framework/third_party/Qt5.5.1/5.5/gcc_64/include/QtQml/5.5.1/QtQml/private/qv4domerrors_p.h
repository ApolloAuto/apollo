/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the QtQml module of the Qt Toolkit.
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

#ifndef QV8DOMERRORS_P_H
#define QV8DOMERRORS_P_H

//
//  W A R N I N G
//  -------------
//
// This file is not part of the Qt API.  It exists purely as an
// implementation detail.  This header file may change from version to
// version without notice, or even be removed.
//
// We mean it.
//

#include <QtCore/qglobal.h>

QT_BEGIN_NAMESPACE
// From DOM-Level-3-Core spec
// http://www.w3.org/TR/DOM-Level-3-Core/core.html
#define DOMEXCEPTION_INDEX_SIZE_ERR 1
#define DOMEXCEPTION_DOMSTRING_SIZE_ERR 2
#define DOMEXCEPTION_HIERARCHY_REQUEST_ERR 3
#define DOMEXCEPTION_WRONG_DOCUMENT_ERR 4
#define DOMEXCEPTION_INVALID_CHARACTER_ERR 5
#define DOMEXCEPTION_NO_DATA_ALLOWED_ERR 6
#define DOMEXCEPTION_NO_MODIFICATION_ALLOWED_ERR 7
#define DOMEXCEPTION_NOT_FOUND_ERR 8
#define DOMEXCEPTION_NOT_SUPPORTED_ERR 9
#define DOMEXCEPTION_INUSE_ATTRIBUTE_ERR 10
#define DOMEXCEPTION_INVALID_STATE_ERR 11
#define DOMEXCEPTION_SYNTAX_ERR 12
#define DOMEXCEPTION_INVALID_MODIFICATION_ERR 13
#define DOMEXCEPTION_NAMESPACE_ERR 14
#define DOMEXCEPTION_INVALID_ACCESS_ERR 15
#define DOMEXCEPTION_VALIDATION_ERR 16
#define DOMEXCEPTION_TYPE_MISMATCH_ERR 17

#define V4THROW_DOM(error, string) { \
    QV4::ScopedValue v(scope, scope.engine->newString(QStringLiteral(string))); \
    QV4::ScopedObject ex(scope, scope.engine->newErrorObject(v)); \
    ex->put(QV4::ScopedString(scope, scope.engine->newIdentifier(QStringLiteral("code"))), QV4::ScopedValue(scope, QV4::Primitive::fromInt32(error))); \
    return ctx->engine()->throwError(ex); \
}

namespace QV4 {
struct ExecutionEngine;
}


void qt_add_domexceptions(QV4::ExecutionEngine *e);

QT_END_NAMESPACE

#endif // QV8DOMERRORS_P_H
