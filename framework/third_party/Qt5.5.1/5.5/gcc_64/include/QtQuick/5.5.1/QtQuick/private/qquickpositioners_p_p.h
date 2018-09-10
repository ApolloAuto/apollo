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

#ifndef QQUICKPOSITIONERS_P_P_H
#define QQUICKPOSITIONERS_P_P_H

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

#include "qquickpositioners_p.h"
#include "qquickimplicitsizeitem_p_p.h"

#include <QtQuick/private/qquickstate_p.h>
#include <private/qquicktransitionmanager_p_p.h>
#include <private/qquickstatechangescript_p.h>

#include <QtCore/qobject.h>
#include <QtCore/qstring.h>
#include <QtCore/qtimer.h>

QT_BEGIN_NAMESPACE

class QQuickItemViewTransitioner;

class QQuickBasePositionerPrivate : public QQuickImplicitSizeItemPrivate, public QQuickItemChangeListener
{
    Q_DECLARE_PUBLIC(QQuickBasePositioner)

public:
    QQuickBasePositionerPrivate()
        : spacing(0), type(QQuickBasePositioner::None)
        , transitioner(0), positioningDirty(false)
        , doingPositioning(false), anchorConflict(false), layoutDirection(Qt::LeftToRight)
    {
    }

    void init(QQuickBasePositioner::PositionerType at)
    {
        type = at;
    }

    qreal spacing;

    QQuickBasePositioner::PositionerType type;
    QQuickItemViewTransitioner *transitioner;

    void watchChanges(QQuickItem *other);
    void unwatchChanges(QQuickItem* other);
    void setPositioningDirty() {
        Q_Q(QQuickBasePositioner);
        if (!positioningDirty) {
            positioningDirty = true;
            q->polish();
        }
    }

    bool positioningDirty : 1;
    bool doingPositioning : 1;
    bool anchorConflict : 1;

    Qt::LayoutDirection layoutDirection;

    void mirrorChange() Q_DECL_OVERRIDE {
        effectiveLayoutDirectionChange();
    }
    bool isLeftToRight() const {
        if (type == QQuickBasePositioner::Vertical)
            return true;
        else
            return effectiveLayoutMirror ? layoutDirection == Qt::RightToLeft : layoutDirection == Qt::LeftToRight;
    }

    void itemSiblingOrderChanged(QQuickItem* other) Q_DECL_OVERRIDE
    {
        Q_UNUSED(other);
        setPositioningDirty();
    }

    void itemGeometryChanged(QQuickItem *, const QRectF &newGeometry, const QRectF &oldGeometry) Q_DECL_OVERRIDE
    {
        if (newGeometry.size() != oldGeometry.size())
            setPositioningDirty();
    }

    void itemVisibilityChanged(QQuickItem *) Q_DECL_OVERRIDE
    {
        setPositioningDirty();
    }

    void itemDestroyed(QQuickItem *item) Q_DECL_OVERRIDE
    {
        Q_Q(QQuickBasePositioner);
        int index = q->positionedItems.find(QQuickBasePositioner::PositionedItem(item));
        if (index >= 0)
            q->removePositionedItem(&q->positionedItems, index);
    }

    static Qt::LayoutDirection getLayoutDirection(const QQuickBasePositioner *positioner)
    {
        return positioner->d_func()->layoutDirection;
    }

    static Qt::LayoutDirection getEffectiveLayoutDirection(const QQuickBasePositioner *positioner)
    {
        if (positioner->d_func()->effectiveLayoutMirror)
            return positioner->d_func()->layoutDirection == Qt::RightToLeft ? Qt::LeftToRight : Qt::RightToLeft;
        else
            return positioner->d_func()->layoutDirection;
    }

    virtual void effectiveLayoutDirectionChange()
    {
    }
};

QT_END_NAMESPACE

#endif // QQUICKPOSITIONERS_P_P_H
