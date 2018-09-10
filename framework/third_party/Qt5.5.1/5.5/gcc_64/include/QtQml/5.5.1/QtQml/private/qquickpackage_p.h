/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the QtQml module of the Qt Toolkit.
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

#ifndef QQUICKPACKAGE_H
#define QQUICKPACKAGE_H

#include <qqml.h>

QT_BEGIN_NAMESPACE

class QQuickPackagePrivate;
class QQuickPackageAttached;
class Q_AUTOTEST_EXPORT QQuickPackage : public QObject
{
    Q_OBJECT
    Q_DECLARE_PRIVATE(QQuickPackage)

    Q_CLASSINFO("DefaultProperty", "data")
    Q_PROPERTY(QQmlListProperty<QObject> data READ data)

public:
    QQuickPackage(QObject *parent=0);
    virtual ~QQuickPackage();

    QQmlListProperty<QObject> data();

    QObject *part(const QString & = QString());
    bool hasPart(const QString &);

    static QQuickPackageAttached *qmlAttachedProperties(QObject *);
};

class QQuickPackageAttached : public QObject
{
Q_OBJECT
Q_PROPERTY(QString name READ name WRITE setName)
public:
    QQuickPackageAttached(QObject *parent);
    virtual ~QQuickPackageAttached();

    QString name() const;
    void setName(const QString &n);

    static QHash<QObject *, QQuickPackageAttached *> attached;
private:
    QString _name;
};

QT_END_NAMESPACE

QML_DECLARE_TYPE(QQuickPackage)
QML_DECLARE_TYPEINFO(QQuickPackage, QML_HAS_ATTACHED_PROPERTIES)

#endif // QQUICKPACKAGE_H
