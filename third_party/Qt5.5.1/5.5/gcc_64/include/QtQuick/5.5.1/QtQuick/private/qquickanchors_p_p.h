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

#ifndef QQUICKANCHORS_P_P_H
#define QQUICKANCHORS_P_P_H

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

#include "qquickanchors_p.h"
#include "qquickitemchangelistener_p.h"
#include <private/qobject_p.h>

QT_BEGIN_NAMESPACE

class QQuickAnchorLine
{
public:
    enum AnchorLine {
        Invalid = 0x0,
        Left = 0x01,
        Right = 0x02,
        Top = 0x04,
        Bottom = 0x08,
        HCenter = 0x10,
        VCenter = 0x20,
        Baseline = 0x40,
        Horizontal_Mask = Left | Right | HCenter,
        Vertical_Mask = Top | Bottom | VCenter | Baseline
    };

    QQuickAnchorLine() : item(0), anchorLine(Invalid) {}
    QQuickAnchorLine(QQuickItem *i, AnchorLine l) : item(i), anchorLine(l) {}

    QQuickItem *item;
    AnchorLine anchorLine;
};

inline bool operator==(const QQuickAnchorLine& a, const QQuickAnchorLine& b)
{
    return a.item == b.item && a.anchorLine == b.anchorLine;
}

class QQuickAnchorsPrivate : public QObjectPrivate, public QQuickItemChangeListener
{
    Q_DECLARE_PUBLIC(QQuickAnchors)
public:
    QQuickAnchorsPrivate(QQuickItem *i)
      : componentComplete(true), updatingMe(false), inDestructor(false), centerAligned(true),
        leftMarginExplicit(false), rightMarginExplicit(false), topMarginExplicit(false),
        bottomMarginExplicit(false), updatingHorizontalAnchor(0),
        updatingVerticalAnchor(0), updatingFill(0), updatingCenterIn(0), item(i), usedAnchors(0), fill(0),
        centerIn(0), leftMargin(0), rightMargin(0), topMargin(0), bottomMargin(0),
        margins(0), vCenterOffset(0), hCenterOffset(0), baselineOffset(0)

    {
    }

    void clearItem(QQuickItem *);

    int calculateDependency(QQuickItem *);
    void addDepend(QQuickItem *);
    void remDepend(QQuickItem *);
    bool isItemComplete() const;

    bool componentComplete:1;
    bool updatingMe:1;
    bool inDestructor:1;
    bool centerAligned:1;
    bool leftMarginExplicit : 1;
    bool rightMarginExplicit : 1;
    bool topMarginExplicit : 1;
    bool bottomMarginExplicit : 1;
    uint updatingHorizontalAnchor:2;
    uint updatingVerticalAnchor:2;
    uint updatingFill:2;
    uint updatingCenterIn:2;

    void setItemHeight(qreal);
    void setItemWidth(qreal);
    void setItemX(qreal);
    void setItemY(qreal);
    void setItemPos(const QPointF &);
    void setItemSize(const QSizeF &);

    void update();
    void updateOnComplete();
    void updateMe();

    // QQuickItemGeometryListener interface
    void itemGeometryChanged(QQuickItem *, const QRectF &, const QRectF &) Q_DECL_OVERRIDE;
    QQuickAnchorsPrivate *anchorPrivate() Q_DECL_OVERRIDE { return this; }

    bool checkHValid() const;
    bool checkVValid() const;
    bool checkHAnchorValid(QQuickAnchorLine anchor) const;
    bool checkVAnchorValid(QQuickAnchorLine anchor) const;
    bool calcStretch(const QQuickAnchorLine &edge1, const QQuickAnchorLine &edge2, qreal offset1, qreal offset2, QQuickAnchorLine::AnchorLine line, qreal &stretch);

    bool isMirrored() const;
    void updateHorizontalAnchors();
    void updateVerticalAnchors();
    void fillChanged();
    void centerInChanged();

    QQuickItem *item;
    QQuickAnchors::Anchors usedAnchors;

    QQuickItem *fill;
    QQuickItem *centerIn;

    QQuickAnchorLine left;
    QQuickAnchorLine right;
    QQuickAnchorLine top;
    QQuickAnchorLine bottom;
    QQuickAnchorLine vCenter;
    QQuickAnchorLine hCenter;
    QQuickAnchorLine baseline;

    qreal leftMargin;
    qreal rightMargin;
    qreal topMargin;
    qreal bottomMargin;
    qreal margins;
    qreal vCenterOffset;
    qreal hCenterOffset;
    qreal baselineOffset;


    static inline QQuickAnchorsPrivate *get(QQuickAnchors *o) {
        return static_cast<QQuickAnchorsPrivate *>(QObjectPrivate::get(o));
    }
};

QT_END_NAMESPACE

Q_DECLARE_METATYPE(QQuickAnchorLine)

#endif
