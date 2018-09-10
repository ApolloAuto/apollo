/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the Qt Designer of the Qt Toolkit.
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

//
//  W A R N I N G
//  -------------
//
// This file is not part of the Qt API.  It exists for the convenience
// of Qt Designer.  This header
// file may change from version to version without notice, or even be removed.
//
// We mean it.
//

#ifndef LAYOUT_H
#define LAYOUT_H

#include "shared_global_p.h"
#include "layoutinfo_p.h"

#include <QtCore/QPointer>
#include <QtCore/QObject>
#include <QtCore/QMap>
#include <QtCore/QHash>

#include <QtWidgets/QLayout>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class QDesignerFormWindowInterface;

namespace qdesigner_internal {
class QDESIGNER_SHARED_EXPORT Layout : public QObject
{
    Q_OBJECT
    Q_DISABLE_COPY(Layout)
protected:
    Layout(const QWidgetList &wl, QWidget *p, QDesignerFormWindowInterface *fw, QWidget *lb, LayoutInfo::Type layoutType);

public:
    static  Layout* createLayout(const QWidgetList &widgets,  QWidget *parentWidget,
                                 QDesignerFormWindowInterface *fw,
                                 QWidget *layoutBase, LayoutInfo::Type layoutType);

    virtual ~Layout();

    virtual void sort() = 0;
    virtual void doLayout() = 0;

    virtual void setup();
    virtual void undoLayout();
    virtual void breakLayout();

    const QWidgetList &widgets() const { return m_widgets; }
    QWidget *parentWidget() const      { return m_parentWidget; }
    QWidget *layoutBaseWidget() const  { return m_layoutBase; }

    /* Determines whether instances of QLayoutWidget are unmanaged/hidden
     * after breaking a layout. Default is true. Can be turned off when
      * morphing */
    bool reparentLayoutWidget() const  { return m_reparentLayoutWidget; }
    void setReparentLayoutWidget(bool v) {  m_reparentLayoutWidget = v; }

protected:
    virtual void finishLayout(bool needMove, QLayout *layout = 0);
    virtual bool prepareLayout(bool &needMove, bool &needReparent);

    void setWidgets(const  QWidgetList &widgets) { m_widgets = widgets; }
    QLayout *createLayout(int type);
    void reparentToLayoutBase(QWidget *w);

private slots:
    void widgetDestroyed();

private:
    QWidgetList m_widgets;
    QWidget *m_parentWidget;
    typedef QHash<QWidget *, QRect> WidgetGeometryHash;
    WidgetGeometryHash m_geometries;
    QWidget *m_layoutBase;
    QDesignerFormWindowInterface *m_formWindow;
    const LayoutInfo::Type m_layoutType;
    QPoint m_startPoint;
    QRect m_oldGeometry;

    bool m_reparentLayoutWidget;
    const bool m_isBreak;
};

namespace Utils
{

inline int indexOfWidget(QLayout *layout, QWidget *widget)
{
    int index = 0;
    while (QLayoutItem *item = layout->itemAt(index)) {
        if (item->widget() == widget)
            return index;

        ++index;
    }

    return -1;
}

} // namespace Utils

} // namespace qdesigner_internal

QT_END_NAMESPACE

#endif // LAYOUT_H
