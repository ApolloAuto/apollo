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

#ifndef ZOOMWIDGET_H
#define ZOOMWIDGET_H

#include "shared_global_p.h"

#include <QtWidgets/QGraphicsView>
#include <QtWidgets/QGraphicsProxyWidget>
#include <QtCore/QList>

QT_BEGIN_NAMESPACE

class QGraphicsScene;
class QMenu;
class QAction;
class QActionGroup;

namespace qdesigner_internal {

// A checkable zoom menu action group. Operates in percent.

class QDESIGNER_SHARED_EXPORT ZoomMenu : public QObject {
    Q_OBJECT
    Q_DISABLE_COPY(ZoomMenu)

public:
    ZoomMenu(QObject *parent = 0);
    void addActions(QMenu *m);

    int zoom() const;

    // Return a list of available zoom values.
    static QList<int> zoomValues();

public slots:
    void setZoom(int percent);

signals:
    void zoomChanged(int);

private slots:
    void slotZoomMenu(QAction *);

private:
    static int zoomOf(const QAction *a);

    QActionGroup *m_menuActions;
};

/* Zoom view: A QGraphicsView with a zoom menu */

class QDESIGNER_SHARED_EXPORT ZoomView : public QGraphicsView
{
    Q_PROPERTY(int zoom READ zoom WRITE setZoom DESIGNABLE true SCRIPTABLE true)
    Q_PROPERTY(bool zoomContextMenuEnabled READ isZoomContextMenuEnabled WRITE setZoomContextMenuEnabled DESIGNABLE true SCRIPTABLE true)
    Q_OBJECT
    Q_DISABLE_COPY(ZoomView)
public:
    ZoomView(QWidget *parent = 0);

    /*  Zoom in percent (for easily implementing menus) and qreal zoomFactor
     * in sync */
    int zoom() const; // in percent
    qreal zoomFactor() const;

    // Zoom Menu on QGraphicsView.
    bool isZoomContextMenuEnabled() const;
    void setZoomContextMenuEnabled(bool e);

    QGraphicsScene &scene() { return *m_scene; }
    const QGraphicsScene &scene() const { return *m_scene; }

    // Helpers for menu
    ZoomMenu *zoomMenu();

    QPoint scrollPosition() const;
    void setScrollPosition(const QPoint& pos);
    void scrollToOrigin();

public slots:
    void setZoom(int percent);
    void showContextMenu(const QPoint &globalPos);

protected:
    void contextMenuEvent(QContextMenuEvent *event);

    // Overwrite for implementing additional behaviour when doing setZoom();
    virtual void applyZoom();

private:
    QGraphicsScene *m_scene;
    int m_zoom;
    qreal m_zoomFactor;

    bool m_zoomContextMenuEnabled;
    bool m_resizeBlocked;
    ZoomMenu *m_zoomMenu;
};

/* The proxy widget used in  ZoomWidget. It  refuses to move away from 0,0,
 * This behaviour is required for Windows only. */

class  QDESIGNER_SHARED_EXPORT ZoomProxyWidget : public QGraphicsProxyWidget {
    Q_DISABLE_COPY(ZoomProxyWidget)
public:
    explicit ZoomProxyWidget(QGraphicsItem *parent = 0, Qt::WindowFlags wFlags = 0);

protected:
    QVariant itemChange(GraphicsItemChange change, const QVariant &value) Q_DECL_OVERRIDE;
};

/* Zoom widget: A QGraphicsView-based container for a widget that allows for
 * zooming it. Communicates changes in size in the following ways:
 * 1) Embedded widget wants to resize: Listen for its resize in event filter
 *    and resize
 * 2) Zoom is changed: resize to fully display the embedded widget
 * 3) Outer widget changes (by manually resizing the window:
 *    Pass the scaled change on to the embedded widget.
 * Provides helper functions for a zoom context menu on the widget. */

class QDESIGNER_SHARED_EXPORT ZoomWidget : public ZoomView
{
    Q_PROPERTY(bool widgetZoomContextMenuEnabled READ isWidgetZoomContextMenuEnabled WRITE setWidgetZoomContextMenuEnabled DESIGNABLE true SCRIPTABLE true)
    Q_PROPERTY(bool itemAcceptDrops READ itemAcceptDrops WRITE setItemAcceptDrops DESIGNABLE true SCRIPTABLE true)
    Q_OBJECT
    Q_DISABLE_COPY(ZoomWidget)

public:
    ZoomWidget(QWidget *parent = 0);
    void setWidget(QWidget *w, Qt::WindowFlags wFlags = 0);

    const QGraphicsProxyWidget *proxy() const { return m_proxy; }
    QGraphicsProxyWidget *proxy() { return m_proxy; }

    /* Enable the zoom Menu on the Widget (as opposed ZoomView's menu which
     * is on the canvas). */
    bool isWidgetZoomContextMenuEnabled() const;
    void setWidgetZoomContextMenuEnabled(bool e);

    void setItemAcceptDrops(bool);
    bool itemAcceptDrops() const;

    QSize minimumSizeHint() const Q_DECL_OVERRIDE;
    QSize sizeHint() const Q_DECL_OVERRIDE;

    bool zoomedEventFilter(QObject *watched, QEvent *event);

public slots:
    // debug state
    void dump() const;

protected:
    void resizeEvent(QResizeEvent * event);

    // Overwritten from ZoomView
    virtual void applyZoom();
    // Overwrite to actually perform a resize. This is required if we are in a layout. Default does resize().
    virtual void doResize(const QSize &s);

private:
    // Factory function for QGraphicsProxyWidgets which can be overwritten. Default creates a ZoomProxyWidget
    virtual QGraphicsProxyWidget *createProxyWidget(QGraphicsItem *parent = 0, Qt::WindowFlags wFlags = 0) const;
    QSize widgetSizeToViewSize(const QSize &s, bool *ptrToValid = 0) const;

    void resizeToWidgetSize();
    QSize viewPortMargin() const;
    QSize widgetSize() const;
    QSizeF widgetDecorationSizeF() const;

    QGraphicsProxyWidget *m_proxy;
    bool m_viewResizeBlocked;
    bool m_widgetResizeBlocked;
    bool m_widgetZoomContextMenuEnabled;
};

} // namespace qdesigner_internal

QT_END_NAMESPACE

#endif
