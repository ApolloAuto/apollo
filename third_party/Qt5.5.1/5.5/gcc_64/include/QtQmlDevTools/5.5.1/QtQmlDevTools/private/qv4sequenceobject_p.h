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

#ifndef QV4SEQUENCEWRAPPER_P_H
#define QV4SEQUENCEWRAPPER_P_H

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
#include <QtCore/qvariant.h>

#include "qv4value_inl_p.h"
#include "qv4object_p.h"
#include "qv4context_p.h"

QT_BEGIN_NAMESPACE

namespace QV4 {

struct SequencePrototype : public QV4::Object
{
    void init();

    static ReturnedValue method_valueOf(QV4::CallContext *ctx)
    {
        return ctx->thisObject().toString(ctx->engine())->asReturnedValue();
    }

    static ReturnedValue method_sort(QV4::CallContext *ctx);

    static bool isSequenceType(int sequenceTypeId);
    static ReturnedValue newSequence(QV4::ExecutionEngine *engine, int sequenceTypeId, QObject *object, int propertyIndex, bool *succeeded);
    static ReturnedValue fromVariant(QV4::ExecutionEngine *engine, const QVariant& v, bool *succeeded);
    static int metaTypeForSequence(Object *object);
    static QVariant toVariant(Object *object);
    static QVariant toVariant(const Value &array, int typeHint, bool *succeeded);
};

}

QT_END_NAMESPACE

#endif // QV4SEQUENCEWRAPPER_P_H
