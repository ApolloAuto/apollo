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

#ifndef QQUICKLOADER_P_H
#define QQUICKLOADER_P_H

#include "qquickimplicitsizeitem_p.h"

QT_BEGIN_NAMESPACE

class QQuickLoaderPrivate;
class Q_AUTOTEST_EXPORT QQuickLoader : public QQuickImplicitSizeItem
{
    Q_OBJECT
    Q_ENUMS(Status)

    Q_PROPERTY(bool active READ active WRITE setActive NOTIFY activeChanged)
    Q_PROPERTY(QUrl source READ source WRITE setSource NOTIFY sourceChanged)
    Q_PROPERTY(QQmlComponent *sourceComponent READ sourceComponent WRITE setSourceComponent RESET resetSourceComponent NOTIFY sourceComponentChanged)
    Q_PROPERTY(QObject *item READ item NOTIFY itemChanged)
    Q_PROPERTY(Status status READ status NOTIFY statusChanged)
    Q_PROPERTY(qreal progress READ progress NOTIFY progressChanged)
    Q_PROPERTY(bool asynchronous READ asynchronous WRITE setAsynchronous NOTIFY asynchronousChanged)

public:
    QQuickLoader(QQuickItem *parent = 0);
    virtual ~QQuickLoader();

    bool active() const;
    void setActive(bool newVal);

    Q_INVOKABLE void setSource(QQmlV4Function *);

    QUrl source() const;
    void setSource(const QUrl &);

    QQmlComponent *sourceComponent() const;
    void setSourceComponent(QQmlComponent *);
    void resetSourceComponent();

    enum Status { Null, Ready, Loading, Error };
    Status status() const;
    qreal progress() const;

    bool asynchronous() const;
    void setAsynchronous(bool a);

    QObject *item() const;

Q_SIGNALS:
    void itemChanged();
    void activeChanged();
    void sourceChanged();
    void sourceComponentChanged();
    void statusChanged();
    void progressChanged();
    void loaded();
    void asynchronousChanged();

protected:
    void geometryChanged(const QRectF &newGeometry, const QRectF &oldGeometry) Q_DECL_OVERRIDE;
    void componentComplete() Q_DECL_OVERRIDE;

private:
    void setSource(const QUrl &sourceUrl, bool needsClear);
    void loadFromSource();
    void loadFromSourceComponent();
    Q_DISABLE_COPY(QQuickLoader)
    Q_DECLARE_PRIVATE(QQuickLoader)
    Q_PRIVATE_SLOT(d_func(), void _q_sourceLoaded())
    Q_PRIVATE_SLOT(d_func(), void _q_updateSize())
};

QT_END_NAMESPACE

QML_DECLARE_TYPE(QQuickLoader)

#endif // QQUICKLOADER_P_H
