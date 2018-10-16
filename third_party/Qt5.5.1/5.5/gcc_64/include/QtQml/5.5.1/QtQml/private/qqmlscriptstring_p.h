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

#ifndef QQMLSCRIPTSTRING_P_H
#define QQMLSCRIPTSTRING_P_H

#include "qqmlscriptstring.h"
#include <QtQml/qqmlcontext.h>

QT_BEGIN_NAMESPACE

class Q_AUTOTEST_EXPORT QQmlScriptStringPrivate : public QSharedData
{
public:
    QQmlScriptStringPrivate() : context(0), scope(0), bindingId(-1), lineNumber(0), columnNumber(0),
        numberValue(0), isStringLiteral(false), isNumberLiteral(false) {}

    //for testing
    static const QQmlScriptStringPrivate* get(const QQmlScriptString &script);

    QQmlContext *context;
    QObject *scope;
    QString script;
    int bindingId;
    quint16 lineNumber;
    quint16 columnNumber;
    double numberValue;
    bool isStringLiteral;
    bool isNumberLiteral;
};

QT_END_NAMESPACE

#endif // QQMLSCRIPTSTRING_P_H
