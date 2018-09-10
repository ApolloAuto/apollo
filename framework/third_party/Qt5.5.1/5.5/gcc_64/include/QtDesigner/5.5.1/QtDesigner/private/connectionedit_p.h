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


#ifndef CONNECTIONEDIT_H
#define CONNECTIONEDIT_H

#include "shared_global_p.h"

#include <QtCore/QMultiMap>
#include <QtCore/QList>
#include <QtCore/QPointer>

#include <QtWidgets/QWidget>
#include <QtGui/QPixmap>
#include <QtGui/QPolygonF>

#include <QtWidgets/QUndoCommand>

QT_BEGIN_NAMESPACE

class QDesignerFormWindowInterface;
class QUndoStack;
class QMenu;

namespace qdesigner_internal {

class Connection;
class ConnectionEdit;

class QDESIGNER_SHARED_EXPORT CETypes
{
public:
    typedef QList<Connection*> ConnectionList;
    typedef QMap<Connection*, Connection*> ConnectionSet;
    typedef QMap<QWidget*, QWidget*> WidgetSet;

    class EndPoint {
    public:
        enum Type { Source, Target };
        explicit EndPoint(Connection *_con = 0, Type _type = Source) : con(_con), type(_type) {}
        bool isNull() const { return con == 0; }
        bool operator == (const EndPoint &other) const { return con == other.con && type == other.type; }
        bool operator != (const EndPoint &other) const { return !operator == (other); }
        Connection *con;
        Type type;
    };
    enum LineDir { UpDir = 0, DownDir, RightDir, LeftDir };
};

class QDESIGNER_SHARED_EXPORT Connection : public CETypes
{
public:
    explicit Connection(ConnectionEdit *edit);
    explicit Connection(ConnectionEdit *edit, QObject *source, QObject *target);
    virtual ~Connection() {}

    QObject *object(EndPoint::Type type) const
    {
        return (type == EndPoint::Source ? m_source : m_target);
    }

    QWidget *widget(EndPoint::Type type) const
    {
        return qobject_cast<QWidget*>(object(type));
    }

    QPoint endPointPos(EndPoint::Type type) const;
    QRect endPointRect(EndPoint::Type) const;
    void setEndPoint(EndPoint::Type type, QObject *w, const QPoint &pos)
        { type == EndPoint::Source ? setSource(w, pos) : setTarget(w, pos); }

    bool isVisible() const;
    virtual void updateVisibility();
    void setVisible(bool b);

    virtual QRegion region() const;
    bool contains(const QPoint &pos) const;
    virtual void paint(QPainter *p) const;

    void update(bool update_widgets = true) const;
    void checkWidgets();

    QString label(EndPoint::Type type) const
        { return type == EndPoint::Source ? m_source_label : m_target_label; }
    void setLabel(EndPoint::Type type, const QString &text);
    QRect labelRect(EndPoint::Type type) const;
    QPixmap labelPixmap(EndPoint::Type type) const
        { return type == EndPoint::Source ? m_source_label_pm : m_target_label_pm; }

    ConnectionEdit *edit() const { return m_edit; }

    virtual void inserted() {}
    virtual void removed() {}

private:
    QPoint m_source_pos, m_target_pos;
    QObject *m_source, *m_target;
    QList<QPoint> m_knee_list;
    QPolygonF m_arrow_head;
    ConnectionEdit *m_edit;
    QString m_source_label, m_target_label;
    QPixmap m_source_label_pm, m_target_label_pm;
    QRect m_source_rect, m_target_rect;
    bool m_visible;

    void setSource(QObject *source, const QPoint &pos);
    void setTarget(QObject *target, const QPoint &pos);
    void updateKneeList();
    void trimLine();
    void updatePixmap(EndPoint::Type type);
    LineDir labelDir(EndPoint::Type type) const;
    bool ground() const;
    QRect groundRect() const;
};

class QDESIGNER_SHARED_EXPORT ConnectionEdit : public QWidget, public CETypes
{
    Q_OBJECT
public:
    ConnectionEdit(QWidget *parent, QDesignerFormWindowInterface *form);
    virtual ~ConnectionEdit();

    inline const QPointer<QWidget> &background() const { return m_bg_widget; }

    void setSelected(Connection *con, bool sel);
    bool selected(const Connection *con) const;

    int connectionCount() const { return m_con_list.size(); }
    Connection *connection(int i) const { return m_con_list.at(i); }
    int indexOfConnection(Connection *con) const { return m_con_list.indexOf(con); }

    virtual void setSource(Connection *con, const QString &obj_name);
    virtual void setTarget(Connection *con, const QString &obj_name);

    QUndoStack *undoStack() const { return m_undo_stack; }

    void clear();

    void showEvent(QShowEvent * /*e*/)
    {
        updateBackground();
    }

signals:
    void aboutToAddConnection(int idx);
    void connectionAdded(Connection *con);
    void aboutToRemoveConnection(Connection *con);
    void connectionRemoved(int idx);
    void connectionSelected(Connection *con);
    void widgetActivated(QWidget *wgt);
    void connectionChanged(Connection *con);

public slots:
    void selectNone();
    void selectAll();
    virtual void deleteSelected();
    virtual void setBackground(QWidget *background);
    virtual void updateBackground();
    virtual void widgetRemoved(QWidget *w);
    virtual void objectRemoved(QObject *o);

    void updateLines();
    void enableUpdateBackground(bool enable);

protected:
    void paintEvent(QPaintEvent *e) Q_DECL_OVERRIDE;
    void mouseMoveEvent(QMouseEvent *e) Q_DECL_OVERRIDE;
    void mousePressEvent(QMouseEvent *e) Q_DECL_OVERRIDE;
    void mouseReleaseEvent(QMouseEvent *e) Q_DECL_OVERRIDE;
    void keyPressEvent(QKeyEvent *e) Q_DECL_OVERRIDE;
    void mouseDoubleClickEvent(QMouseEvent *e) Q_DECL_OVERRIDE;
    void resizeEvent(QResizeEvent *e) Q_DECL_OVERRIDE;
    void contextMenuEvent(QContextMenuEvent * event) Q_DECL_OVERRIDE;

    virtual Connection *createConnection(QWidget *source, QWidget *target);
    virtual void modifyConnection(Connection *con);

    virtual QWidget *widgetAt(const QPoint &pos) const;
    virtual void createContextMenu(QMenu &menu);
    void addConnection(Connection *con);
    QRect widgetRect(QWidget *w) const;

    enum State { Editing, Connecting, Dragging };
    State state() const;

    virtual void endConnection(QWidget *target, const QPoint &pos);

    const ConnectionList &connectionList() const { return m_con_list; }
    const ConnectionSet &selection()  const      { return m_sel_con_set; }
    Connection *takeConnection(Connection *con);
    Connection *newlyAddedConnection()           { return m_tmp_con; }
    void clearNewlyAddedConnection();

    void findObjectsUnderMouse(const QPoint &pos);

private:
    void startConnection(QWidget *source, const QPoint &pos);
    void continueConnection(QWidget *target, const QPoint &pos);
    void abortConnection();

    void startDrag(const EndPoint &end_point, const QPoint &pos);
    void continueDrag(const QPoint &pos);
    void endDrag(const QPoint &pos);
    void adjustHotSopt(const EndPoint &end_point, const QPoint &pos);
    Connection *connectionAt(const QPoint &pos) const;
    EndPoint endPointAt(const QPoint &pos) const;
    void paintConnection(QPainter *p, Connection *con,
                         WidgetSet *heavy_highlight_set,
                         WidgetSet *light_highlight_set) const;
    void paintLabel(QPainter *p, EndPoint::Type type, Connection *con);


    QPointer<QWidget> m_bg_widget;
    QUndoStack *m_undo_stack;
    bool m_enable_update_background;

    Connection *m_tmp_con; // the connection we are currently editing
    ConnectionList m_con_list;
    bool m_start_connection_on_drag;
    EndPoint m_end_point_under_mouse;
    QPointer<QWidget> m_widget_under_mouse;

    EndPoint m_drag_end_point;
    QPoint m_old_source_pos, m_old_target_pos;
    ConnectionSet m_sel_con_set;
    const QColor m_inactive_color;
    const QColor m_active_color;

private:
    friend class Connection;
    friend class AddConnectionCommand;
    friend class DeleteConnectionsCommand;
    friend class SetEndPointCommand;
};

class QDESIGNER_SHARED_EXPORT CECommand : public QUndoCommand, public CETypes
{
public:
   explicit  CECommand(ConnectionEdit *edit)
        : m_edit(edit) {}

    virtual bool mergeWith(const QUndoCommand *) { return false; }

    ConnectionEdit *edit() const { return m_edit; }

private:
    ConnectionEdit *m_edit;
};

class QDESIGNER_SHARED_EXPORT AddConnectionCommand : public CECommand
{
public:
    AddConnectionCommand(ConnectionEdit *edit, Connection *con);
    virtual void redo();
    virtual void undo();
private:
    Connection *m_con;
};

class QDESIGNER_SHARED_EXPORT DeleteConnectionsCommand : public CECommand
{
public:
    DeleteConnectionsCommand(ConnectionEdit *edit, const ConnectionList &con_list);
    virtual void redo();
    virtual void undo();
private:
    ConnectionList m_con_list;
};

} // namespace qdesigner_internal

QT_END_NAMESPACE

#endif // CONNECTIONEDIT_H
