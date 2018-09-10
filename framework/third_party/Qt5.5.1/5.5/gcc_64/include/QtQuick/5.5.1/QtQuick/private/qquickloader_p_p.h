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

#ifndef QQUICKLOADER_P_P_H
#define QQUICKLOADER_P_P_H

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

#include "qquickloader_p.h"
#include "qquickimplicitsizeitem_p_p.h"
#include "qquickitemchangelistener_p.h"
#include <qqmlincubator.h>

#include <private/qv4value_inl_p.h>

QT_BEGIN_NAMESPACE


class QQuickLoaderPrivate;
class QQuickLoaderIncubator : public QQmlIncubator
{
public:
    QQuickLoaderIncubator(QQuickLoaderPrivate *l, IncubationMode mode) : QQmlIncubator(mode), loader(l) {}

protected:
    void statusChanged(Status) Q_DECL_OVERRIDE;
    void setInitialState(QObject *) Q_DECL_OVERRIDE;

private:
    QQuickLoaderPrivate *loader;
};

class QQmlContext;
class QQuickLoaderPrivate : public QQuickImplicitSizeItemPrivate, public QQuickItemChangeListener
{
    Q_DECLARE_PUBLIC(QQuickLoader)

public:
    QQuickLoaderPrivate();
    ~QQuickLoaderPrivate();

    void itemGeometryChanged(QQuickItem *item, const QRectF &newGeometry, const QRectF &oldGeometry) Q_DECL_OVERRIDE;
    void itemImplicitWidthChanged(QQuickItem *) Q_DECL_OVERRIDE;
    void itemImplicitHeightChanged(QQuickItem *) Q_DECL_OVERRIDE;
    void clear();
    void initResize();
    void load();

    void incubatorStateChanged(QQmlIncubator::Status status);
    void setInitialState(QObject *o);
    void disposeInitialPropertyValues();
    QUrl resolveSourceUrl(QQmlV4Function *args);
    QV4::ReturnedValue extractInitialPropertyValues(QQmlV4Function *args, QObject *loader, bool *error);

    qreal getImplicitWidth() const Q_DECL_OVERRIDE;
    qreal getImplicitHeight() const Q_DECL_OVERRIDE;

    QUrl source;
    QQuickItem *item;
    QObject *object;
    QQmlComponent *component;
    QV4::PersistentValue componentStrongReference; // To ensure GC doesn't delete components created by Qt.createComponent
    QQmlContext *itemContext;
    QQuickLoaderIncubator *incubator;
    QV4::PersistentValue initialPropertyValues;
    QV4::PersistentValue qmlGlobalForIpv;
    bool updatingSize: 1;
    bool active : 1;
    bool loadingFromSource : 1;
    bool asynchronous : 1;

    void _q_sourceLoaded();
    void _q_updateSize(bool loaderGeometryChanged = true);
};

QT_END_NAMESPACE

#endif // QQUICKLOADER_P_P_H
