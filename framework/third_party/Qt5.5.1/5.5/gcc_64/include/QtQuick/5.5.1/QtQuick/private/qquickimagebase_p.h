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

#ifndef QQUICKIMAGEBASE_P_H
#define QQUICKIMAGEBASE_P_H

#include "qquickimplicitsizeitem_p.h"
#include <private/qtquickglobal_p.h>

QT_BEGIN_NAMESPACE

class QQuickImageBasePrivate;
class Q_QUICK_PRIVATE_EXPORT QQuickImageBase : public QQuickImplicitSizeItem
{
    Q_OBJECT
    Q_ENUMS(Status)

    Q_PROPERTY(Status status READ status NOTIFY statusChanged)
    Q_PROPERTY(QUrl source READ source WRITE setSource NOTIFY sourceChanged)
    Q_PROPERTY(qreal progress READ progress NOTIFY progressChanged)
    Q_PROPERTY(bool asynchronous READ asynchronous WRITE setAsynchronous NOTIFY asynchronousChanged)
    Q_PROPERTY(bool cache READ cache WRITE setCache NOTIFY cacheChanged)
    Q_PROPERTY(QSize sourceSize READ sourceSize WRITE setSourceSize RESET resetSourceSize NOTIFY sourceSizeChanged)
    Q_PROPERTY(bool mirror READ mirror WRITE setMirror NOTIFY mirrorChanged)

public:
    QQuickImageBase(QQuickItem *parent=0);
    ~QQuickImageBase();
    enum Status { Null, Ready, Loading, Error };
    Status status() const;
    qreal progress() const;

    QUrl source() const;
    virtual void setSource(const QUrl &url);

    bool asynchronous() const;
    void setAsynchronous(bool);

    bool cache() const;
    void setCache(bool);

    QImage image() const;

    virtual void setSourceSize(const QSize&);
    QSize sourceSize() const;
    void resetSourceSize();

    virtual void setMirror(bool mirror);
    bool mirror() const;

    virtual void setAutoTransform(bool transform);
    bool autoTransform() const;

    void resolve2xLocalFile(const QUrl &url, qreal targetDevicePixelRatio, QUrl *sourceUrl, qreal *sourceDevicePixelRatio);

    // Use a virtual rather than a signal->signal to avoid the huge
    // connect/conneciton overhead for this rare case.
    virtual void emitAutoTransformBaseChanged() { }

Q_SIGNALS:
    void sourceChanged(const QUrl &);
    void sourceSizeChanged();
    void statusChanged(QQuickImageBase::Status);
    void progressChanged(qreal progress);
    void asynchronousChanged();
    void cacheChanged();
    void mirrorChanged();

protected:
    virtual void load();
    void componentComplete() Q_DECL_OVERRIDE;
    virtual void pixmapChange();
    void itemChange(ItemChange change, const ItemChangeData &value) Q_DECL_OVERRIDE;
    QQuickImageBase(QQuickImageBasePrivate &dd, QQuickItem *parent);

private Q_SLOTS:
    virtual void requestFinished();
    void requestProgress(qint64,qint64);
    void handleScreenChanged(QScreen *screen);

private:
    Q_DISABLE_COPY(QQuickImageBase)
    Q_DECLARE_PRIVATE(QQuickImageBase)
};

QT_END_NAMESPACE

#endif // QQUICKIMAGEBASE_P_H
