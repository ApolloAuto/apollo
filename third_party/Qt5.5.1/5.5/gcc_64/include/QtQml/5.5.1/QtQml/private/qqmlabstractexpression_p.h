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

#ifndef QQMLABSTRACTEXPRESSION_P_H
#define QQMLABSTRACTEXPRESSION_P_H

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

#include <private/qqmlcontext_p.h>
#include <private/qfieldlist_p.h>
#include <private/qflagpointer_p.h>

QT_BEGIN_NAMESPACE

class Q_QML_PRIVATE_EXPORT QQmlAbstractExpression
{
public:
    QQmlAbstractExpression();
    virtual ~QQmlAbstractExpression();

    bool isValid() const;

    QQmlContextData *context() const;
    void setContext(QQmlContextData *);

    virtual void refresh();

    class DeleteWatcher {
    public:
        inline DeleteWatcher(QQmlAbstractExpression *);
        inline ~DeleteWatcher();
        inline bool wasDeleted() const;
    private:
        friend class QQmlAbstractExpression;
        QQmlContextData *_c;
        QQmlAbstractExpression **_w;
        QQmlAbstractExpression *_s;
    };

private:
    friend class QQmlContext;
    friend class QQmlContextData;
    friend class QQmlContextPrivate;

    QBiPointer<QQmlContextData, DeleteWatcher> m_context;
    QQmlAbstractExpression **m_prevExpression;
    QQmlAbstractExpression  *m_nextExpression;
};

QQmlAbstractExpression::DeleteWatcher::DeleteWatcher(QQmlAbstractExpression *e)
: _c(0), _w(0), _s(e)
{
    if (e->m_context.isT1()) {
        _w = &_s;
        _c = e->m_context.asT1();
        e->m_context = this;
    } else {
        // Another watcher is already registered
        _w = &e->m_context.asT2()->_s;
    }
}

QQmlAbstractExpression::DeleteWatcher::~DeleteWatcher()
{
    Q_ASSERT(*_w == 0 || (*_w == _s && _s->m_context.isT2()));
    if (*_w && _s->m_context.asT2() == this)
        _s->m_context = _c;
}

bool QQmlAbstractExpression::DeleteWatcher::wasDeleted() const
{
    return *_w == 0;
}

QT_END_NAMESPACE

#endif // QQMLABSTRACTEXPRESSION_P_H
