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

#ifndef QQUICKWIDGET_H
#define QQUICKWIDGET_H

#include <QtWidgets/qwidget.h>
#include <QtQuick/qquickwindow.h>
#include <QtCore/qurl.h>
#include <QtQml/qqmldebug.h>
#include <QtQuickWidgets/qtquickwidgetsglobal.h>
#include <QtGui/qimage.h>

QT_BEGIN_NAMESPACE

class QQmlEngine;
class QQmlContext;
class QQmlError;
class QQuickItem;
class QQmlComponent;

class QQuickWidgetPrivate;
class Q_QUICKWIDGETS_EXPORT QQuickWidget : public QWidget
{
    Q_OBJECT
    Q_PROPERTY(ResizeMode resizeMode READ resizeMode WRITE setResizeMode)
    Q_PROPERTY(Status status READ status NOTIFY statusChanged)
    Q_PROPERTY(QUrl source READ source WRITE setSource DESIGNABLE true)
    Q_ENUMS(ResizeMode Status)

public:
    explicit QQuickWidget(QWidget *parent = 0);
    QQuickWidget(QQmlEngine* engine, QWidget *parent);
    QQuickWidget(const QUrl &source, QWidget *parent = 0);
    virtual ~QQuickWidget();

    QUrl source() const;

    QQmlEngine* engine() const;
    QQmlContext* rootContext() const;

    QQuickItem *rootObject() const;

    enum ResizeMode { SizeViewToRootObject, SizeRootObjectToView };
    ResizeMode resizeMode() const;
    void setResizeMode(ResizeMode);

    enum Status { Null, Ready, Loading, Error };
    Status status() const;

    QList<QQmlError> errors() const;

    QSize sizeHint() const;
    QSize initialSize() const;

    void setFormat(const QSurfaceFormat &format);
    QSurfaceFormat format() const;

    QImage grabFramebuffer() const;

    void setClearColor(const QColor &color);

    QQuickWindow *quickWindow() const;

public Q_SLOTS:
    void setSource(const QUrl&);
    void setContent(const QUrl& url, QQmlComponent *component, QObject *item);

Q_SIGNALS:
    void statusChanged(QQuickWidget::Status);
    void sceneGraphError(QQuickWindow::SceneGraphError error, const QString &message);

private Q_SLOTS:
    // ### Qt 6: make these truly private slots through Q_PRIVATE_SLOT
    void continueExecute();
    void createFramebufferObject();
    void destroyFramebufferObject();
    void triggerUpdate();

protected:
    virtual void resizeEvent(QResizeEvent *);
    virtual void timerEvent(QTimerEvent*);

    virtual void keyPressEvent(QKeyEvent *);
    virtual void keyReleaseEvent(QKeyEvent *);
    virtual void mousePressEvent(QMouseEvent *);
    virtual void mouseReleaseEvent(QMouseEvent *);
    virtual void mouseMoveEvent(QMouseEvent *);
    virtual void mouseDoubleClickEvent(QMouseEvent *);

    virtual void showEvent(QShowEvent *);
    virtual void hideEvent(QHideEvent *);

    virtual void focusInEvent(QFocusEvent * event);
    virtual void focusOutEvent(QFocusEvent * event);

#ifndef QT_NO_WHEELEVENT
    virtual void wheelEvent(QWheelEvent *);
#endif

#ifndef QT_NO_DRAGANDDROP
    virtual void dragEnterEvent(QDragEnterEvent *);
    virtual void dragMoveEvent(QDragMoveEvent *);
    virtual void dragLeaveEvent(QDragLeaveEvent *);
    virtual void dropEvent(QDropEvent *);
#endif

    bool event(QEvent *);

private:
    Q_DISABLE_COPY(QQuickWidget)
    Q_DECLARE_PRIVATE(QQuickWidget)
};

QT_END_NAMESPACE

#endif // QQuickWidget_H
