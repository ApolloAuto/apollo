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

#ifndef QQUICKVIEW_P_H
#define QQUICKVIEW_P_H

#include "qquickview.h"

#include <QtCore/qurl.h>
#include <QtCore/qelapsedtimer.h>
#include <QtCore/qtimer.h>
#include <QtCore/qpointer.h>
#include <QtCore/QWeakPointer>

#include <QtQml/qqmlengine.h>
#include <private/qv4object_p.h>
#include "qquickwindow_p.h"

#include "qquickitemchangelistener_p.h"

QT_BEGIN_NAMESPACE

namespace QV4 {
struct ExecutionEngine;
}

class QQmlContext;
class QQmlError;
class QQuickItem;
class QQmlComponent;

class QQuickViewPrivate : public QQuickWindowPrivate,
                       public QQuickItemChangeListener
{
    Q_DECLARE_PUBLIC(QQuickView)
public:
    static QQuickViewPrivate* get(QQuickView *view) { return view->d_func(); }
    static const QQuickViewPrivate* get(const QQuickView *view) { return view->d_func(); }

    QQuickViewPrivate();
    ~QQuickViewPrivate();

    void execute();
    void itemGeometryChanged(QQuickItem *item, const QRectF &newGeometry, const QRectF &oldGeometry) Q_DECL_OVERRIDE;
    void initResize();
    void updateSize();
    void setRootObject(QObject *);

    void init(QQmlEngine* e = 0);

    QSize rootObjectSize() const;

    QPointer<QQuickItem> root;

    QUrl source;

    QPointer<QQmlEngine> engine;
    QQmlComponent *component;
    QBasicTimer resizetimer;

    QQuickView::ResizeMode resizeMode;
    QSize initialSize;
    QElapsedTimer frameTimer;
    QV4::PersistentValue rootItemMarker;
};

namespace QV4 {
namespace Heap {

struct QQuickRootItemMarker : Object {
    inline QQuickRootItemMarker(QV4::ExecutionEngine *engine, QQuickWindow *window);

    QQuickWindow *window;
};

}

struct QQuickRootItemMarker : public Object
{
    V4_OBJECT2(QQuickRootItemMarker, Object)

    static Heap::QQuickRootItemMarker *create(QQmlEngine *engine, QQuickWindow *window);

    static void markObjects(QV4::Heap::Base *that, QV4::ExecutionEngine *e);

};

inline
Heap::QQuickRootItemMarker::QQuickRootItemMarker(QV4::ExecutionEngine *engine, QQuickWindow *window)
    : Heap::Object(engine)
    , window(window)
{
}

}

QT_END_NAMESPACE

#endif // QQUICKVIEW_P_H
