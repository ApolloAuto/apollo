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

#ifndef QSGCONTEXTPLUGIN_H
#define QSGCONTEXTPLUGIN_H

#include <private/qtquickglobal_p.h>
#include <QtQuick/qquickimageprovider.h>
#include <QtCore/qplugin.h>
#include <QtCore/qfactoryinterface.h>

QT_BEGIN_NAMESPACE

class QSGContext;

class QSGRenderLoop;

struct Q_QUICK_PRIVATE_EXPORT QSGContextFactoryInterface : public QFactoryInterface
{
    virtual QSGContext *create(const QString &key) const = 0;

    virtual QQuickTextureFactory *createTextureFactoryFromImage(const QImage &image) = 0;
    virtual QSGRenderLoop *createWindowManager() = 0;
};

#define QSGContextFactoryInterface_iid \
        "org.qt-project.Qt.QSGContextFactoryInterface"
Q_DECLARE_INTERFACE(QSGContextFactoryInterface, QSGContextFactoryInterface_iid)

class Q_QUICK_PRIVATE_EXPORT QSGContextPlugin : public QObject, public QSGContextFactoryInterface
{
    Q_OBJECT
    Q_INTERFACES(QSGContextFactoryInterface:QFactoryInterface)
public:
    explicit QSGContextPlugin(QObject *parent = 0);
    virtual ~QSGContextPlugin();

    virtual QStringList keys() const = 0;
    virtual QSGContext *create(const QString &key) const = 0;

    virtual QQuickTextureFactory *createTextureFactoryFromImage(const QImage &) { return 0; }
    virtual QSGRenderLoop *createWindowManager() { return 0; }
};

QT_END_NAMESPACE

#endif // QSGCONTEXTPLUGIN_H
