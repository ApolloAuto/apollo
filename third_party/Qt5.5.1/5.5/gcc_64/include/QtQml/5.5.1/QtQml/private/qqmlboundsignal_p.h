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

#ifndef QQMLBOUNDSIGNAL_P_H
#define QQMLBOUNDSIGNAL_P_H

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

#include <QtCore/qmetaobject.h>

#include <private/qqmlabstractexpression_p.h>
#include <private/qqmljavascriptexpression_p.h>
#include <private/qqmlboundsignalexpressionpointer_p.h>
#include <private/qqmlnotifier_p.h>
#include <private/qflagpointer_p.h>
#include <private/qqmlrefcount_p.h>
#include <private/qqmlglobal_p.h>
#include <private/qbitfield_p.h>

QT_BEGIN_NAMESPACE

class Q_QML_PRIVATE_EXPORT QQmlBoundSignalExpression : public QQmlAbstractExpression, public QQmlJavaScriptExpression, public QQmlRefCount
{
public:
    QQmlBoundSignalExpression(QObject *target, int index,
                              QQmlContextData *ctxt, QObject *scope, const QString &expression,
                              const QString &fileName, quint16 line, quint16 column,
                              const QString &handlerName = QString(),
                              const QString &parameterString = QString());

    QQmlBoundSignalExpression(QObject *target, int index,
                              QQmlContextData *ctxt, QObject *scope, const QV4::Value &function);

    QQmlBoundSignalExpression(QObject *target, int index,
                              QQmlContextData *ctxt, QObject *scope, QV4::Function *runtimeFunction);

    // "inherited" from QQmlJavaScriptExpression.
    static QString expressionIdentifier(QQmlJavaScriptExpression *);
    static void expressionChanged(QQmlJavaScriptExpression *);

    // evaluation of a bound signal expression doesn't return any value
    void evaluate(void **a);

    QQmlSourceLocation sourceLocation() const;
    QString expression() const;
    QV4::Function *function() const;
    QObject *target() const { return m_target; }

    QQmlEngine *engine() const { return context() ? context()->engine : 0; }

private:
    ~QQmlBoundSignalExpression();

    void init(QQmlContextData *ctxt, QObject *scope);

    bool expressionFunctionValid() const { return m_extra.flag(); }
    void setExpressionFunctionValid(bool v) { m_extra.setFlagValue(v); }

    bool invalidParameterName() const { return m_extra.flag2(); }
    void setInvalidParameterName(bool v) { m_extra.setFlag2Value(v); }

    int m_index;
    QV4::PersistentValue m_function;

    QObject *m_target;

    // only needed when !expressionFunctionValid()
    struct ExtraData {
        ExtraData(const QString &handlerName, const QString &parameterString,
                  const QString &expression, const QString &fileName,
                  quint16 line, quint16 column);
        QString m_handlerName;
        QString m_parameterString;
        QString m_expression;
        QQmlSourceLocation m_sourceLocation;
        QV4::PersistentValue m_v8qmlscope;
    };

    // We store some flag bits in the following flag pointers.
    //    flag  - expressionFunctionValid
    //    flag2 - invalidParameterName
    QFlagPointer<ExtraData> m_extra;
};

class Q_QML_PRIVATE_EXPORT QQmlAbstractBoundSignal
{
public:
    QQmlAbstractBoundSignal();
    virtual ~QQmlAbstractBoundSignal();

    virtual int index() const = 0;
    virtual QQmlBoundSignalExpression *expression() const = 0;
    virtual QQmlBoundSignalExpressionPointer setExpression(QQmlBoundSignalExpression *) = 0;
    virtual QQmlBoundSignalExpressionPointer takeExpression(QQmlBoundSignalExpression *) = 0;
    virtual bool isEvaluating() const = 0;

    void removeFromObject();
protected:
    void addToObject(QObject *owner);

private:
    friend class QQmlData;
    friend class QQmlPropertyPrivate;
    friend class QQmlEngineDebugService;
    QQmlAbstractBoundSignal **m_prevSignal;
    QQmlAbstractBoundSignal  *m_nextSignal;
};

class Q_QML_PRIVATE_EXPORT QQmlBoundSignal : public QQmlAbstractBoundSignal,
                                             public QQmlNotifierEndpoint
{
public:
    QQmlBoundSignal(QObject *target, int signal, QObject *owner, QQmlEngine *engine);
    virtual ~QQmlBoundSignal();

    int index() const;

    QQmlBoundSignalExpression *expression() const;
    QQmlBoundSignalExpressionPointer setExpression(QQmlBoundSignalExpression *);
    QQmlBoundSignalExpressionPointer takeExpression(QQmlBoundSignalExpression *);

    bool isEvaluating() const { return m_isEvaluating; }

private:
    friend void QQmlBoundSignal_callback(QQmlNotifierEndpoint *, void **);

    QQmlBoundSignalExpressionPointer m_expression;
    int m_index;
    bool m_isEvaluating;
};

QT_END_NAMESPACE

#endif // QQMLBOUNDSIGNAL_P_H
