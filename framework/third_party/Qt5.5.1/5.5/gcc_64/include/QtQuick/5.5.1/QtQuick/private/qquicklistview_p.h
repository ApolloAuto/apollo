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

#ifndef QQUICKLISTVIEW_P_H
#define QQUICKLISTVIEW_P_H

#include "qquickitemview_p.h"

QT_BEGIN_NAMESPACE

class QQuickListView;
class QQuickListViewPrivate;
class Q_AUTOTEST_EXPORT QQuickViewSection : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QString property READ property WRITE setProperty NOTIFY propertyChanged)
    Q_PROPERTY(SectionCriteria criteria READ criteria WRITE setCriteria NOTIFY criteriaChanged)
    Q_PROPERTY(QQmlComponent *delegate READ delegate WRITE setDelegate NOTIFY delegateChanged)
    Q_PROPERTY(int labelPositioning READ labelPositioning WRITE setLabelPositioning NOTIFY labelPositioningChanged)
    Q_ENUMS(SectionCriteria)
    Q_ENUMS(LabelPositioning)
public:
    QQuickViewSection(QQuickListView *parent=0);

    QString property() const { return m_property; }
    void setProperty(const QString &);

    enum SectionCriteria { FullString, FirstCharacter };
    SectionCriteria criteria() const { return m_criteria; }
    void setCriteria(SectionCriteria);

    QQmlComponent *delegate() const { return m_delegate; }
    void setDelegate(QQmlComponent *delegate);

    QString sectionString(const QString &value);

    enum LabelPositioning { InlineLabels = 0x01, CurrentLabelAtStart = 0x02, NextLabelAtEnd = 0x04 };
    int labelPositioning() { return m_labelPositioning; }
    void setLabelPositioning(int pos);

Q_SIGNALS:
    void sectionsChanged();
    void propertyChanged();
    void criteriaChanged();
    void delegateChanged();
    void labelPositioningChanged();

private:
    QString m_property;
    SectionCriteria m_criteria;
    QQmlComponent *m_delegate;
    int m_labelPositioning;
    QQuickListViewPrivate *m_view;
};


class QQmlInstanceModel;
class QQuickListViewAttached;
class Q_AUTOTEST_EXPORT QQuickListView : public QQuickItemView
{
    Q_OBJECT
    Q_DECLARE_PRIVATE(QQuickListView)

    Q_PROPERTY(qreal highlightMoveVelocity READ highlightMoveVelocity WRITE setHighlightMoveVelocity NOTIFY highlightMoveVelocityChanged)
    Q_PROPERTY(qreal highlightResizeVelocity READ highlightResizeVelocity WRITE setHighlightResizeVelocity NOTIFY highlightResizeVelocityChanged)
    Q_PROPERTY(int highlightResizeDuration READ highlightResizeDuration WRITE setHighlightResizeDuration NOTIFY highlightResizeDurationChanged)

    Q_PROPERTY(qreal spacing READ spacing WRITE setSpacing NOTIFY spacingChanged)
    Q_PROPERTY(Orientation orientation READ orientation WRITE setOrientation NOTIFY orientationChanged)

    Q_PROPERTY(QQuickViewSection *section READ sectionCriteria CONSTANT)
    Q_PROPERTY(QString currentSection READ currentSection NOTIFY currentSectionChanged)

    Q_PROPERTY(SnapMode snapMode READ snapMode WRITE setSnapMode NOTIFY snapModeChanged)

    Q_PROPERTY(HeaderPositioning headerPositioning READ headerPositioning WRITE setHeaderPositioning NOTIFY headerPositioningChanged REVISION 2)
    Q_PROPERTY(FooterPositioning footerPositioning READ footerPositioning WRITE setFooterPositioning NOTIFY footerPositioningChanged REVISION 2)

    Q_ENUMS(Orientation)
    Q_ENUMS(SnapMode)
    Q_ENUMS(HeaderPositioning)
    Q_ENUMS(FooterPositioning)
    Q_CLASSINFO("DefaultProperty", "data")

public:
    QQuickListView(QQuickItem *parent=0);
    ~QQuickListView();

    qreal spacing() const;
    void setSpacing(qreal spacing);

    enum Orientation { Horizontal = Qt::Horizontal, Vertical = Qt::Vertical };
    Orientation orientation() const;
    void setOrientation(Orientation);

    QQuickViewSection *sectionCriteria();
    QString currentSection() const;

    void setHighlightFollowsCurrentItem(bool) Q_DECL_OVERRIDE;

    qreal highlightMoveVelocity() const;
    void setHighlightMoveVelocity(qreal);

    qreal highlightResizeVelocity() const;
    void setHighlightResizeVelocity(qreal);

    int highlightResizeDuration() const;
    void setHighlightResizeDuration(int);

    void setHighlightMoveDuration(int) Q_DECL_OVERRIDE;

    enum SnapMode { NoSnap, SnapToItem, SnapOneItem };
    SnapMode snapMode() const;
    void setSnapMode(SnapMode mode);

    enum HeaderPositioning { InlineHeader, OverlayHeader, PullBackHeader };
    HeaderPositioning headerPositioning() const;
    void setHeaderPositioning(HeaderPositioning positioning);

    enum FooterPositioning { InlineFooter, OverlayFooter, PullBackFooter };
    FooterPositioning footerPositioning() const;
    void setFooterPositioning(FooterPositioning positioning);

    static QQuickListViewAttached *qmlAttachedProperties(QObject *);

public Q_SLOTS:
    void incrementCurrentIndex();
    void decrementCurrentIndex();

Q_SIGNALS:
    void spacingChanged();
    void orientationChanged();
    void currentSectionChanged();
    void highlightMoveVelocityChanged();
    void highlightResizeVelocityChanged();
    void highlightResizeDurationChanged();
    void snapModeChanged();
    Q_REVISION(2) void headerPositioningChanged();
    Q_REVISION(2) void footerPositioningChanged();

protected:
    void viewportMoved(Qt::Orientations orient) Q_DECL_OVERRIDE;
    void keyPressEvent(QKeyEvent *) Q_DECL_OVERRIDE;
    void geometryChanged(const QRectF &newGeometry,const QRectF &oldGeometry) Q_DECL_OVERRIDE;
    void initItem(int index, QObject *item) Q_DECL_OVERRIDE;
};

class QQuickListViewAttached : public QQuickItemViewAttached
{
    Q_OBJECT

public:
    QQuickListViewAttached(QObject *parent)
        : QQuickItemViewAttached(parent), m_sectionItem(0) {}
    ~QQuickListViewAttached() {}

public:
    QQuickItem *m_sectionItem;
};


QT_END_NAMESPACE

QML_DECLARE_TYPEINFO(QQuickListView, QML_HAS_ATTACHED_PROPERTIES)
QML_DECLARE_TYPE(QQuickListView)
QML_DECLARE_TYPE(QQuickViewSection)

#endif // QQUICKLISTVIEW_P_H
