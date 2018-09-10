/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the QtQuick module of the Qt Toolkit.
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

#ifndef QQUICKTEXTUTIL_P_H
#define QQUICKTEXTUTIL_P_H

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

#include <QtQml/qqml.h>
#include <QtQml/qqmlincubator.h>
#include <QtQuick/qquickitem.h>

QT_BEGIN_NAMESPACE

class QQuickTextUtil : public QObject   // For the benefit of translations.
{
    Q_OBJECT
public:
    template <typename Private> static void setCursorDelegate(Private *d, QQmlComponent *delegate);
    template <typename Private> static void createCursor(Private *d);


    static qreal alignedX(qreal textWidth, qreal itemWidth, int alignment);
    static qreal alignedY(qreal textHeight, qreal itemHeight, int alignment);

private:
    static QQuickItem *createCursor(
            QQmlComponent *component,
            QQuickItem *parent,
            const QRectF &cursorRectangle,
            const char *className);

};

template <typename Private>
void QQuickTextUtil::setCursorDelegate(Private *d, QQmlComponent *delegate)
{
    if (d->cursorComponent == delegate)
        return;

    typename Private::Public *parent = d->q_func();

    if (d->cursorComponent) {
        disconnect(d->cursorComponent, SIGNAL(statusChanged(QQmlComponent::Status)),
                parent, SLOT(createCursor()));
    }

    delete d->cursorItem;
    d->cursorItem = 0;
    d->cursorPending = true;

    d->cursorComponent = delegate;

    if (parent->isCursorVisible() && parent->isComponentComplete())
        createCursor(d);

    Q_EMIT parent->cursorDelegateChanged();
}

template <typename Private>
void QQuickTextUtil::createCursor(Private *d)
{
    if (!d->cursorPending)
        return;

    d->cursorPending = false;

    typename Private::Public *parent = d->q_func();
    if (d->cursorComponent) {
        d->cursorItem = createCursor(
                d->cursorComponent,
                parent,
                parent->cursorRectangle(),
                Private::Public::staticMetaObject.className());
    }

    d->setNativeCursorEnabled(!d->cursorItem);
    d->updateType = Private::UpdatePaintNode;
    parent->update();
}

QT_END_NAMESPACE

#endif
