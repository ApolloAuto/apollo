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

#ifndef QQMLEXPRESSION_P_H
#define QQMLEXPRESSION_P_H

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

#include "qqmlexpression.h"

#include <private/qqmlengine_p.h>
#include <private/qfieldlist_p.h>
#include <private/qflagpointer_p.h>
#include <private/qdeletewatcher_p.h>
#include <private/qpointervaluepair_p.h>
#include <private/qqmlabstractexpression_p.h>
#include <private/qqmljavascriptexpression_p.h>

QT_BEGIN_NAMESPACE

class QQmlExpression;
class QString;
class QQmlExpressionPrivate : public QObjectPrivate,
                              public QQmlJavaScriptExpression,
                              public QQmlAbstractExpression
{
    Q_DECLARE_PUBLIC(QQmlExpression)
public:
    QQmlExpressionPrivate();
    ~QQmlExpressionPrivate();

    void init(QQmlContextData *, const QString &, QObject *);
    void init(QQmlContextData *, QV4::Function *runtimeFunction, QObject *);

    QVariant value(bool *isUndefined = 0);

    QV4::ReturnedValue v4value(bool *isUndefined = 0);

    static inline QQmlExpressionPrivate *get(QQmlExpression *expr);
    static inline QQmlExpression *get(QQmlExpressionPrivate *expr);

    void _q_notify();

    bool expressionFunctionValid:1;

    // "Inherited" from QQmlJavaScriptExpression
    static QString expressionIdentifier(QQmlJavaScriptExpression *);
    static void expressionChanged(QQmlJavaScriptExpression *);
    virtual void expressionChanged();

    QString expression;

    QV4::PersistentValue qmlscope;
    QV4::PersistentValue function;

    QString url; // This is a QString for a reason.  QUrls are slooooooow...
    quint16 line;
    quint16 column;
    QString name; //function name, hint for the debugger
};

QQmlExpressionPrivate *QQmlExpressionPrivate::get(QQmlExpression *expr)
{
    return static_cast<QQmlExpressionPrivate *>(QObjectPrivate::get(expr));
}

QQmlExpression *QQmlExpressionPrivate::get(QQmlExpressionPrivate *expr)
{
    return expr->q_func();
}


QT_END_NAMESPACE

#endif // QQMLEXPRESSION_P_H
