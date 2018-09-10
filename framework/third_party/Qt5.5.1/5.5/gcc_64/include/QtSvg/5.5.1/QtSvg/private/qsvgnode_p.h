/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the Qt SVG module of the Qt Toolkit.
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

#ifndef QSVGNODE_P_H
#define QSVGNODE_P_H

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

#include "qsvgstyle_p.h"
#include "qtsvgglobal_p.h"

#include "QtCore/qstring.h"
#include "QtCore/qhash.h"

QT_BEGIN_NAMESPACE

class QPainter;
class QSvgTinyDocument;

class Q_SVG_PRIVATE_EXPORT QSvgNode
{
public:
    enum Type
    {
        DOC,
        G,
        DEFS,
        SWITCH,
        ANIMATION,
        ARC,
        CIRCLE,
        ELLIPSE,
        IMAGE,
        LINE,
        PATH,
        POLYGON,
        POLYLINE,
        RECT,
        TEXT,
        TEXTAREA,
        TSPAN,
        USE,
        VIDEO
    };
    enum DisplayMode {
        InlineMode,
        BlockMode,
        ListItemMode,
        RunInMode,
        CompactMode,
        MarkerMode,
        TableMode,
        InlineTableMode,
        TableRowGroupMode,
        TableHeaderGroupMode,
        TableFooterGroupMode,
        TableRowMode,
        TableColumnGroupMode,
        TableColumnMode,
        TableCellMode,
        TableCaptionMode,
        NoneMode,
        InheritMode
    };
public:
    QSvgNode(QSvgNode *parent=0);
    virtual ~QSvgNode();
    virtual void draw(QPainter *p, QSvgExtraStates &states) =0;

    QSvgNode *parent() const;

    void appendStyleProperty(QSvgStyleProperty *prop, const QString &id);
    void applyStyle(QPainter *p, QSvgExtraStates &states) const;
    void revertStyle(QPainter *p, QSvgExtraStates &states) const;
    QSvgStyleProperty *styleProperty(QSvgStyleProperty::Type type) const;
    QSvgFillStyleProperty *styleProperty(const QString &id) const;

    QSvgTinyDocument *document() const;

    virtual Type type() const =0;
    virtual QRectF bounds(QPainter *p, QSvgExtraStates &states) const;
    virtual QRectF transformedBounds(QPainter *p, QSvgExtraStates &states) const;
    QRectF transformedBounds() const;

    void setRequiredFeatures(const QStringList &lst);
    const QStringList & requiredFeatures() const;

    void setRequiredExtensions(const QStringList &lst);
    const QStringList & requiredExtensions() const;

    void setRequiredLanguages(const QStringList &lst);
    const QStringList & requiredLanguages() const;

    void setRequiredFormats(const QStringList &lst);
    const QStringList & requiredFormats() const;

    void setRequiredFonts(const QStringList &lst);
    const QStringList & requiredFonts() const;

    void setVisible(bool visible);
    bool isVisible() const;

    void setDisplayMode(DisplayMode display);
    DisplayMode displayMode() const;

    QString nodeId() const;
    void setNodeId(const QString &i);

    QString xmlClass() const;
    void setXmlClass(const QString &str);
protected:
    mutable QSvgStyle m_style;

    static qreal strokeWidth(QPainter *p);
private:
    QSvgNode   *m_parent;

    QStringList m_requiredFeatures;
    QStringList m_requiredExtensions;
    QStringList m_requiredLanguages;
    QStringList m_requiredFormats;
    QStringList m_requiredFonts;

    bool        m_visible;

    QString m_id;
    QString m_class;

    DisplayMode m_displayMode;
    mutable QRectF m_cachedBounds;

    friend class QSvgTinyDocument;
};

inline QSvgNode *QSvgNode::parent() const
{
    return m_parent;
}

inline bool QSvgNode::isVisible() const
{
    return m_visible;
}

inline QString QSvgNode::nodeId() const
{
    return m_id;
}

inline QString QSvgNode::xmlClass() const
{
    return m_class;
}

QT_END_NAMESPACE

#endif // QSVGNODE_P_H
