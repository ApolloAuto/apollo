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

#ifndef QQUICKVIEW_H
#define QQUICKVIEW_H

#include <QtQuick/qquickwindow.h>
#include <QtCore/qurl.h>
#include <QtQml/qqmldebug.h>

QT_BEGIN_NAMESPACE

class QQmlEngine;
class QQmlContext;
class QQmlError;
class QQuickItem;
class QQmlComponent;

class QQuickViewPrivate;
class Q_QUICK_EXPORT QQuickView : public QQuickWindow
{
    Q_OBJECT
    Q_PROPERTY(ResizeMode resizeMode READ resizeMode WRITE setResizeMode)
    Q_PROPERTY(Status status READ status NOTIFY statusChanged)
    Q_PROPERTY(QUrl source READ source WRITE setSource DESIGNABLE true)
    Q_ENUMS(ResizeMode Status)
public:
    explicit QQuickView(QWindow *parent = 0);
    QQuickView(QQmlEngine* engine, QWindow *parent);
    QQuickView(const QUrl &source, QWindow *parent = 0);
    virtual ~QQuickView();

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

public Q_SLOTS:
    void setSource(const QUrl&);
    void setContent(const QUrl& url, QQmlComponent *component, QObject *item);

Q_SIGNALS:
    void statusChanged(QQuickView::Status);

private Q_SLOTS:
    void continueExecute();

protected:
    void resizeEvent(QResizeEvent *) Q_DECL_OVERRIDE;
    void timerEvent(QTimerEvent*) Q_DECL_OVERRIDE;

    void keyPressEvent(QKeyEvent *) Q_DECL_OVERRIDE;
    void keyReleaseEvent(QKeyEvent *) Q_DECL_OVERRIDE;
    void mousePressEvent(QMouseEvent *) Q_DECL_OVERRIDE;
    void mouseReleaseEvent(QMouseEvent *) Q_DECL_OVERRIDE;
    void mouseMoveEvent(QMouseEvent *) Q_DECL_OVERRIDE;
private:
    Q_DISABLE_COPY(QQuickView)
    Q_DECLARE_PRIVATE(QQuickView)
};

QT_END_NAMESPACE

#endif // QQUICKVIEW_H
