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

#ifndef QQMLLISTMODEL_P_P_H
#define QQMLLISTMODEL_P_P_H

//
//  W A R N I N G
//  -------------
//
// This file is not part of the Qt API.  It exists purely as an
// implementation detail.  This header file may change from version to
// version without notice, or even be removed.
//
// We mean it.
//

#include "qqmllistmodel_p.h"
#include <private/qqmlengine_p.h>
#include <private/qqmlopenmetaobject_p.h>
#include <qqml.h>

QT_BEGIN_NAMESPACE


class DynamicRoleModelNode;

class DynamicRoleModelNodeMetaObject : public QQmlOpenMetaObject
{
public:
    DynamicRoleModelNodeMetaObject(DynamicRoleModelNode *object);
    ~DynamicRoleModelNodeMetaObject();

    bool m_enabled;

protected:
    void propertyWrite(int index);
    void propertyWritten(int index);

private:
    DynamicRoleModelNode *m_owner;
};

class DynamicRoleModelNode : public QObject
{
    Q_OBJECT
public:
    DynamicRoleModelNode(QQmlListModel *owner, int uid);

    static DynamicRoleModelNode *create(const QVariantMap &obj, QQmlListModel *owner);

    void updateValues(const QVariantMap &object, QVector<int> &roles);

    QVariant getValue(const QString &name)
    {
        return m_meta->value(name.toUtf8());
    }

    bool setValue(const QByteArray &name, const QVariant &val)
    {
        return m_meta->setValue(name, val);
    }

    void setNodeUpdatesEnabled(bool enable)
    {
        m_meta->m_enabled = enable;
    }

    int getUid() const
    {
        return m_uid;
    }

    static void sync(DynamicRoleModelNode *src, DynamicRoleModelNode *target, QHash<int, QQmlListModel *> *targetModelHash);

private:
    QQmlListModel *m_owner;
    int m_uid;
    DynamicRoleModelNodeMetaObject *m_meta;

    friend class DynamicRoleModelNodeMetaObject;
};

class ModelObject;

class ModelNodeMetaObject : public QQmlOpenMetaObject
{
public:
    ModelNodeMetaObject(ModelObject *object);
    ~ModelNodeMetaObject();

    bool m_enabled;

protected:
    void propertyWritten(int index);

private:

    ModelObject *m_obj;
};

class ModelObject : public QObject
{
    Q_OBJECT
public:
    ModelObject(QQmlListModel *model, int elementIndex);

    void setValue(const QByteArray &name, const QVariant &val, bool force)
    {
        if (force) {
            QVariant existingValue = m_meta->value(name);
            if (existingValue.isValid()) {
                (*m_meta)[name] = QVariant();
            }
        }
        m_meta->setValue(name, val);
    }

    void setNodeUpdatesEnabled(bool enable)
    {
        m_meta->m_enabled = enable;
    }

    void updateValues();
    void updateValues(const QVector<int> &roles);

    QQmlListModel *m_model;
    int m_elementIndex;

private:
    ModelNodeMetaObject *m_meta;
};

class ListLayout
{
public:
    ListLayout() : currentBlock(0), currentBlockOffset(0) {}
    ListLayout(const ListLayout *other);
    ~ListLayout();

    class Role
    {
    public:

        Role() : type(Invalid), blockIndex(-1), blockOffset(-1), index(-1), subLayout(0) {}
        explicit Role(const Role *other);
        ~Role();

        // This enum must be kept in sync with the roleTypeNames variable in qdeclarativelistmodel.cpp
        enum DataType
        {
            Invalid = -1,

            String,
            Number,
            Bool,
            List,
            QObject,
            VariantMap,
            DateTime,

            MaxDataType
        };

        QString name;
        DataType type;
        int blockIndex;
        int blockOffset;
        int index;
        ListLayout *subLayout;
    };

    const Role *getRoleOrCreate(const QString &key, const QVariant &data);
    const Role &getRoleOrCreate(QV4::String *key, Role::DataType type);
    const Role &getRoleOrCreate(const QString &key, Role::DataType type);

    const Role &getExistingRole(int index) { return *roles.at(index); }
    const Role *getExistingRole(const QString &key);
    const Role *getExistingRole(QV4::String *key);

    int roleCount() const { return roles.count(); }

    static void sync(ListLayout *src, ListLayout *target);

private:
    const Role &createRole(const QString &key, Role::DataType type);

    int currentBlock;
    int currentBlockOffset;
    QVector<Role *> roles;
    QStringHash<Role *> roleHash;
};

/*!
\internal
*/
class ListElement
{
public:

    ListElement();
    ListElement(int existingUid);
    ~ListElement();

    static void sync(ListElement *src, ListLayout *srcLayout, ListElement *target, ListLayout *targetLayout, QHash<int, ListModel *> *targetModelHash);

    enum
    {
        BLOCK_SIZE = 64 - sizeof(int) - sizeof(ListElement *) - sizeof(ModelObject *)
    };

private:

    void destroy(ListLayout *layout);

    int setVariantProperty(const ListLayout::Role &role, const QVariant &d);

    int setJsProperty(const ListLayout::Role &role, const QV4::Value &d, QV4::ExecutionEngine *eng);

    int setStringProperty(const ListLayout::Role &role, const QString &s);
    int setDoubleProperty(const ListLayout::Role &role, double n);
    int setBoolProperty(const ListLayout::Role &role, bool b);
    int setListProperty(const ListLayout::Role &role, ListModel *m);
    int setQObjectProperty(const ListLayout::Role &role, QObject *o);
    int setVariantMapProperty(const ListLayout::Role &role, QV4::Object *o);
    int setVariantMapProperty(const ListLayout::Role &role, QVariantMap *m);
    int setDateTimeProperty(const ListLayout::Role &role, const QDateTime &dt);

    void setStringPropertyFast(const ListLayout::Role &role, const QString &s);
    void setDoublePropertyFast(const ListLayout::Role &role, double n);
    void setBoolPropertyFast(const ListLayout::Role &role, bool b);
    void setQObjectPropertyFast(const ListLayout::Role &role, QObject *o);
    void setListPropertyFast(const ListLayout::Role &role, ListModel *m);
    void setVariantMapFast(const ListLayout::Role &role, QV4::Object *o);
    void setDateTimePropertyFast(const ListLayout::Role &role, const QDateTime &dt);

    void clearProperty(const ListLayout::Role &role);

    QVariant getProperty(const ListLayout::Role &role, const QQmlListModel *owner, QV4::ExecutionEngine *eng);
    ListModel *getListProperty(const ListLayout::Role &role);
    QString *getStringProperty(const ListLayout::Role &role);
    QObject *getQObjectProperty(const ListLayout::Role &role);
    QPointer<QObject> *getGuardProperty(const ListLayout::Role &role);
    QVariantMap *getVariantMapProperty(const ListLayout::Role &role);
    QDateTime *getDateTimeProperty(const ListLayout::Role &role);

    inline char *getPropertyMemory(const ListLayout::Role &role);

    int getUid() const { return uid; }

    char data[BLOCK_SIZE];
    ListElement *next;

    int uid;
    ModelObject *m_objectCache;

    friend class ListModel;
};

/*!
\internal
*/
class ListModel
{
public:

    ListModel(ListLayout *layout, QQmlListModel *modelCache, int uid);
    ~ListModel() {}

    void destroy();

    int setOrCreateProperty(int elementIndex, const QString &key, const QVariant &data);
    int setExistingProperty(int uid, const QString &key, const QV4::Value &data, QV4::ExecutionEngine *eng);

    QVariant getProperty(int elementIndex, int roleIndex, const QQmlListModel *owner, QV4::ExecutionEngine *eng);
    ListModel *getListProperty(int elementIndex, const ListLayout::Role &role);

    int roleCount() const
    {
        return m_layout->roleCount();
    }

    const ListLayout::Role &getExistingRole(int index)
    {
        return m_layout->getExistingRole(index);
    }

    const ListLayout::Role &getOrCreateListRole(const QString &name)
    {
        return m_layout->getRoleOrCreate(name, ListLayout::Role::List);
    }

    int elementCount() const
    {
        return elements.count();
    }

    void set(int elementIndex, QV4::Object *object, QVector<int> *roles);
    void set(int elementIndex, QV4::Object *object);

    int append(QV4::Object *object);
    void insert(int elementIndex, QV4::Object *object);

    void clear();
    void remove(int index, int count);

    int appendElement();
    void insertElement(int index);

    void move(int from, int to, int n);

    int getUid() const { return m_uid; }

    static void sync(ListModel *src, ListModel *target, QHash<int, ListModel *> *srcModelHash);

    ModelObject *getOrCreateModelObject(QQmlListModel *model, int elementIndex);

private:
    QPODVector<ListElement *, 4> elements;
    ListLayout *m_layout;
    int m_uid;

    QQmlListModel *m_modelCache;

    struct ElementSync
    {
        ElementSync() : src(0), target(0) {}

        ListElement *src;
        ListElement *target;
    };

    void newElement(int index);

    void updateCacheIndices();

    friend class ListElement;
    friend class QQmlListModelWorkerAgent;
};

QT_END_NAMESPACE

Q_DECLARE_METATYPE(ListModel *);

#endif // QQUICKLISTMODEL_P_P_H
