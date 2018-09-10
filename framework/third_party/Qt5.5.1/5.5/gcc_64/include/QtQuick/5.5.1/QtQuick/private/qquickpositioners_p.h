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

#ifndef QQUICKPOSITIONERS_P_H
#define QQUICKPOSITIONERS_P_H

#include "qquickimplicitsizeitem_p.h"
#include "qquickitemviewtransition_p.h"

#include <QtQuick/private/qquickstate_p.h>
#include <private/qpodvector_p.h>

#include <QtCore/qobject.h>
#include <QtCore/qstring.h>

QT_BEGIN_NAMESPACE

class QQuickBasePositionerPrivate;

class QQuickPositionerAttached : public QObject
{
    Q_OBJECT

public:
    QQuickPositionerAttached(QObject *parent);

    Q_PROPERTY(int index READ index NOTIFY indexChanged)
    Q_PROPERTY(bool isFirstItem READ isFirstItem NOTIFY isFirstItemChanged)
    Q_PROPERTY(bool isLastItem READ isLastItem NOTIFY isLastItemChanged)

    int index() const { return m_index; }
    void setIndex(int index);

    bool isFirstItem() const { return m_isFirstItem; }
    void setIsFirstItem(bool isFirstItem);

    bool isLastItem() const { return m_isLastItem; }
    void setIsLastItem(bool isLastItem);

Q_SIGNALS:
    void indexChanged();
    void isFirstItemChanged();
    void isLastItemChanged();

private:
    int m_index;
    bool m_isFirstItem;
    bool m_isLastItem;
};

class Q_QUICK_PRIVATE_EXPORT QQuickBasePositioner : public QQuickImplicitSizeItem
{
    Q_OBJECT

    Q_PROPERTY(qreal spacing READ spacing WRITE setSpacing NOTIFY spacingChanged)
    Q_PROPERTY(QQuickTransition *populate READ populate WRITE setPopulate NOTIFY populateChanged)
    Q_PROPERTY(QQuickTransition *move READ move WRITE setMove NOTIFY moveChanged)
    Q_PROPERTY(QQuickTransition *add READ add WRITE setAdd NOTIFY addChanged)
public:
    enum PositionerType { None = 0x0, Horizontal = 0x1, Vertical = 0x2, Both = 0x3 };

    QQuickBasePositioner(PositionerType, QQuickItem *parent);
    ~QQuickBasePositioner();

    qreal spacing() const;
    void setSpacing(qreal);

    QQuickTransition *populate() const;
    void setPopulate(QQuickTransition *);

    QQuickTransition *move() const;
    void setMove(QQuickTransition *);

    QQuickTransition *add() const;
    void setAdd(QQuickTransition *);

    static QQuickPositionerAttached *qmlAttachedProperties(QObject *obj);

    void updateAttachedProperties(QQuickPositionerAttached *specificProperty = 0, QQuickItem *specificPropertyOwner = 0) const;

protected:
    QQuickBasePositioner(QQuickBasePositionerPrivate &dd, PositionerType at, QQuickItem *parent);
    void componentComplete() Q_DECL_OVERRIDE;
    void itemChange(ItemChange, const ItemChangeData &) Q_DECL_OVERRIDE;

    void updatePolish() Q_DECL_OVERRIDE;

Q_SIGNALS:
    void spacingChanged();
    void populateChanged();
    void moveChanged();
    void addChanged();

protected Q_SLOTS:
    void prePositioning();

protected:
    virtual void doPositioning(QSizeF *contentSize)=0;
    virtual void reportConflictingAnchors()=0;

    class PositionedItem
    {
    public :
        PositionedItem(QQuickItem *i);
        ~PositionedItem();
        bool operator==(const PositionedItem &other) const { return other.item == item; }

        qreal itemX() const;
        qreal itemY() const;

        void moveTo(const QPointF &pos);

        void transitionNextReposition(QQuickItemViewTransitioner *transitioner, QQuickItemViewTransitioner::TransitionType type, bool asTarget);
        bool prepareTransition(QQuickItemViewTransitioner *transitioner, const QRectF &viewBounds);
        void startTransition(QQuickItemViewTransitioner *transitioner);

        QQuickItem *item;
        QQuickItemViewTransitionableItem *transitionableItem;
        int index;
        bool isNew;
        bool isVisible;
    };

    QPODVector<PositionedItem,8> positionedItems;
    QPODVector<PositionedItem,8> unpositionedItems;//Still 'in' the positioner, just not positioned

    void positionItem(qreal x, qreal y, PositionedItem *target);
    void positionItemX(qreal, PositionedItem *target);
    void positionItemY(qreal, PositionedItem *target);

    void removePositionedItem(QPODVector<PositionedItem,8> *items, int index);
    void clearPositionedItems(QPODVector<PositionedItem,8> *items);

private:
    Q_DISABLE_COPY(QQuickBasePositioner)
    Q_DECLARE_PRIVATE(QQuickBasePositioner)
};

class Q_AUTOTEST_EXPORT QQuickColumn : public QQuickBasePositioner
{
    Q_OBJECT
public:
    QQuickColumn(QQuickItem *parent=0);

protected:
    void doPositioning(QSizeF *contentSize) Q_DECL_OVERRIDE;
    void reportConflictingAnchors() Q_DECL_OVERRIDE;
private:
    Q_DISABLE_COPY(QQuickColumn)
};

class QQuickRowPrivate;
class Q_AUTOTEST_EXPORT QQuickRow: public QQuickBasePositioner
{
    Q_OBJECT
    Q_PROPERTY(Qt::LayoutDirection layoutDirection READ layoutDirection WRITE setLayoutDirection NOTIFY layoutDirectionChanged)
    Q_PROPERTY(Qt::LayoutDirection effectiveLayoutDirection READ effectiveLayoutDirection NOTIFY effectiveLayoutDirectionChanged)
public:
    QQuickRow(QQuickItem *parent=0);

    Qt::LayoutDirection layoutDirection() const;
    void setLayoutDirection (Qt::LayoutDirection);
    Qt::LayoutDirection effectiveLayoutDirection() const;

Q_SIGNALS:
    void layoutDirectionChanged();
    void effectiveLayoutDirectionChanged();

protected:
    void doPositioning(QSizeF *contentSize) Q_DECL_OVERRIDE;
    void reportConflictingAnchors() Q_DECL_OVERRIDE;
private:
    Q_DISABLE_COPY(QQuickRow)
    Q_DECLARE_PRIVATE(QQuickRow)
};

class QQuickGridPrivate;
class Q_AUTOTEST_EXPORT QQuickGrid : public QQuickBasePositioner
{
    Q_OBJECT
    Q_PROPERTY(int rows READ rows WRITE setRows NOTIFY rowsChanged)
    Q_PROPERTY(int columns READ columns WRITE setColumns NOTIFY columnsChanged)
    Q_PROPERTY(qreal rowSpacing READ rowSpacing WRITE setRowSpacing NOTIFY rowSpacingChanged RESET resetRowSpacing)
    Q_PROPERTY(qreal columnSpacing READ columnSpacing WRITE setColumnSpacing NOTIFY columnSpacingChanged RESET resetColumnSpacing)
    Q_PROPERTY(Flow flow READ flow WRITE setFlow NOTIFY flowChanged)
    Q_PROPERTY(Qt::LayoutDirection layoutDirection READ layoutDirection WRITE setLayoutDirection NOTIFY layoutDirectionChanged)
    Q_PROPERTY(Qt::LayoutDirection effectiveLayoutDirection READ effectiveLayoutDirection NOTIFY effectiveLayoutDirectionChanged)
    Q_PROPERTY(HAlignment horizontalItemAlignment READ hItemAlign WRITE setHItemAlign NOTIFY horizontalAlignmentChanged REVISION 1)
    Q_PROPERTY(HAlignment effectiveHorizontalItemAlignment READ effectiveHAlign NOTIFY effectiveHorizontalAlignmentChanged REVISION 1)
    Q_PROPERTY(VAlignment verticalItemAlignment READ vItemAlign WRITE setVItemAlign NOTIFY verticalAlignmentChanged REVISION 1)

public:
    QQuickGrid(QQuickItem *parent=0);

    int rows() const { return m_rows; }
    void setRows(const int rows);

    int columns() const { return m_columns; }
    void setColumns(const int columns);

    qreal rowSpacing() const { return m_rowSpacing; }
    void setRowSpacing(qreal);
    void resetRowSpacing() { m_useRowSpacing = false; }

    qreal columnSpacing() const { return m_columnSpacing; }
    void setColumnSpacing(qreal);
    void resetColumnSpacing() { m_useColumnSpacing = false; }

    Q_ENUMS(Flow)
    enum Flow { LeftToRight, TopToBottom };
    Flow flow() const;
    void setFlow(Flow);

    Qt::LayoutDirection layoutDirection() const;
    void setLayoutDirection (Qt::LayoutDirection);
    Qt::LayoutDirection effectiveLayoutDirection() const;

    Q_ENUMS(HAlignment)
    Q_ENUMS(VAlignment)
    enum HAlignment { AlignLeft = Qt::AlignLeft,
                       AlignRight = Qt::AlignRight,
                       AlignHCenter = Qt::AlignHCenter};
    enum VAlignment { AlignTop = Qt::AlignTop,
                       AlignBottom = Qt::AlignBottom,
                       AlignVCenter = Qt::AlignVCenter };

    HAlignment hItemAlign() const;
    void setHItemAlign(HAlignment align);
    HAlignment effectiveHAlign() const;

    VAlignment vItemAlign() const;
    void setVItemAlign(VAlignment align);

Q_SIGNALS:
    void rowsChanged();
    void columnsChanged();
    void flowChanged();
    void layoutDirectionChanged();
    void effectiveLayoutDirectionChanged();
    void rowSpacingChanged();
    void columnSpacingChanged();
    Q_REVISION(1) void horizontalAlignmentChanged(HAlignment alignment);
    Q_REVISION(1) void effectiveHorizontalAlignmentChanged(HAlignment alignment);
    Q_REVISION(1) void verticalAlignmentChanged(VAlignment alignment);

protected:
    void doPositioning(QSizeF *contentSize) Q_DECL_OVERRIDE;
    void reportConflictingAnchors() Q_DECL_OVERRIDE;

private:
    int m_rows;
    int m_columns;
    qreal m_rowSpacing;
    qreal m_columnSpacing;
    bool m_useRowSpacing;
    bool m_useColumnSpacing;
    Flow m_flow;
    HAlignment m_hItemAlign;
    VAlignment m_vItemAlign;
    Q_DISABLE_COPY(QQuickGrid)
    Q_DECLARE_PRIVATE(QQuickGrid)
};

class QQuickFlowPrivate;
class Q_AUTOTEST_EXPORT QQuickFlow: public QQuickBasePositioner
{
    Q_OBJECT
    Q_PROPERTY(Flow flow READ flow WRITE setFlow NOTIFY flowChanged)
    Q_PROPERTY(Qt::LayoutDirection layoutDirection READ layoutDirection WRITE setLayoutDirection NOTIFY layoutDirectionChanged)
    Q_PROPERTY(Qt::LayoutDirection effectiveLayoutDirection READ effectiveLayoutDirection NOTIFY effectiveLayoutDirectionChanged)
public:
    QQuickFlow(QQuickItem *parent=0);

    Q_ENUMS(Flow)
    enum Flow { LeftToRight, TopToBottom };
    Flow flow() const;
    void setFlow(Flow);

    Qt::LayoutDirection layoutDirection() const;
    void setLayoutDirection (Qt::LayoutDirection);
    Qt::LayoutDirection effectiveLayoutDirection() const;

Q_SIGNALS:
    void flowChanged();
    void layoutDirectionChanged();
    void effectiveLayoutDirectionChanged();

protected:
    void doPositioning(QSizeF *contentSize) Q_DECL_OVERRIDE;
    void reportConflictingAnchors() Q_DECL_OVERRIDE;
protected:
    QQuickFlow(QQuickFlowPrivate &dd, QQuickItem *parent);
private:
    Q_DISABLE_COPY(QQuickFlow)
    Q_DECLARE_PRIVATE(QQuickFlow)
};


QT_END_NAMESPACE

QML_DECLARE_TYPE(QQuickColumn)
QML_DECLARE_TYPE(QQuickRow)
QML_DECLARE_TYPE(QQuickGrid)
QML_DECLARE_TYPE(QQuickFlow)

QML_DECLARE_TYPE(QQuickBasePositioner)
QML_DECLARE_TYPEINFO(QQuickBasePositioner, QML_HAS_ATTACHED_PROPERTIES)

#endif // QQUICKPOSITIONERS_P_H
