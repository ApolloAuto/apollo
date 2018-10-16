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

#ifndef QQMLINSTANCEMODEL_P_H
#define QQMLINSTANCEMODEL_P_H

#include <private/qtqmlglobal_p.h>
#include <QtQml/qqml.h>
#include <QtCore/qobject.h>

QT_BEGIN_NAMESPACE

class QObject;
class QQmlChangeSet;

class Q_QML_PRIVATE_EXPORT QQmlInstanceModel : public QObject
{
    Q_OBJECT

    Q_PROPERTY(int count READ count NOTIFY countChanged)

public:
    virtual ~QQmlInstanceModel() {}

    enum ReleaseFlag { Referenced = 0x01, Destroyed = 0x02 };
    Q_DECLARE_FLAGS(ReleaseFlags, ReleaseFlag)

    virtual int count() const = 0;
    virtual bool isValid() const = 0;
    virtual QObject *object(int index, bool asynchronous=false) = 0;
    virtual ReleaseFlags release(QObject *object) = 0;
    virtual void cancel(int) {}
    virtual QString stringValue(int, const QString &) = 0;
    virtual void setWatchedRoles(QList<QByteArray> roles) = 0;

    virtual int indexOf(QObject *object, QObject *objectContext) const = 0;

Q_SIGNALS:
    void countChanged();
    void modelUpdated(const QQmlChangeSet &changeSet, bool reset);
    void createdItem(int index, QObject *object);
    void initItem(int index, QObject *object);
    void destroyingItem(QObject *object);

protected:
    QQmlInstanceModel(QObjectPrivate &dd, QObject *parent = 0)
        : QObject(dd, parent) {}

private:
    Q_DISABLE_COPY(QQmlInstanceModel)
};

class QQmlObjectModelAttached;
class QQmlObjectModelPrivate;
class Q_QML_PRIVATE_EXPORT QQmlObjectModel : public QQmlInstanceModel
{
    Q_OBJECT
    Q_DECLARE_PRIVATE(QQmlObjectModel)

    Q_PROPERTY(QQmlListProperty<QObject> children READ children NOTIFY childrenChanged DESIGNABLE false)
    Q_CLASSINFO("DefaultProperty", "children")

public:
    QQmlObjectModel(QObject *parent=0);
    virtual ~QQmlObjectModel() {}

    virtual int count() const;
    virtual bool isValid() const;
    virtual QObject *object(int index, bool asynchronous=false);
    virtual ReleaseFlags release(QObject *object);
    virtual QString stringValue(int index, const QString &role);
    virtual void setWatchedRoles(QList<QByteArray>) {}

    virtual int indexOf(QObject *object, QObject *objectContext) const;

    QQmlListProperty<QObject> children();

    static QQmlObjectModelAttached *qmlAttachedProperties(QObject *obj);

Q_SIGNALS:
    void childrenChanged();

private:
    Q_DISABLE_COPY(QQmlObjectModel)
};

class QQmlObjectModelAttached : public QObject
{
    Q_OBJECT

public:
    QQmlObjectModelAttached(QObject *parent)
        : QObject(parent), m_index(0) {}
    ~QQmlObjectModelAttached() {
        attachedProperties.remove(parent());
    }

    Q_PROPERTY(int index READ index NOTIFY indexChanged)
    int index() const { return m_index; }
    void setIndex(int idx) {
        if (m_index != idx) {
            m_index = idx;
            Q_EMIT indexChanged();
        }
    }

    static QQmlObjectModelAttached *properties(QObject *obj) {
        QQmlObjectModelAttached *rv = attachedProperties.value(obj);
        if (!rv) {
            rv = new QQmlObjectModelAttached(obj);
            attachedProperties.insert(obj, rv);
        }
        return rv;
    }

Q_SIGNALS:
    void indexChanged();

public:
    int m_index;

    static QHash<QObject*, QQmlObjectModelAttached*> attachedProperties;
};


QT_END_NAMESPACE

QML_DECLARE_TYPE(QQmlInstanceModel)
QML_DECLARE_TYPE(QQmlObjectModel)
QML_DECLARE_TYPEINFO(QQmlObjectModel, QML_HAS_ATTACHED_PROPERTIES)

#endif // QQMLINSTANCEMODEL_P_H
