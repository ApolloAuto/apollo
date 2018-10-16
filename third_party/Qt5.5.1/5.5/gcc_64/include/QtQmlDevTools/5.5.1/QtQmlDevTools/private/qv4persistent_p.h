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
#ifndef QV4PERSISTENT_H
#define QV4PERSISTENT_H

#include "qv4value_inl_p.h"

QT_BEGIN_NAMESPACE

namespace QV4 {

struct Q_QML_EXPORT PersistentValueStorage
{
    PersistentValueStorage(ExecutionEngine *engine);
    ~PersistentValueStorage();

    Value *allocate();
    static void free(Value *e);

    void mark(ExecutionEngine *e);

    struct Iterator {
        Q_DECL_CONSTEXPR Iterator(void *p, int idx)
            : p(p), index(idx) {}
        void *p;
        int index;
        Iterator &operator++();
        bool operator !=(const Iterator &other) {
            return p != other.p || index != other.index;
        }
        Value &operator *();
    };
    Iterator begin() { return Iterator(firstPage, 0); }
    Iterator end() { return Iterator(0, 0); }

    static ExecutionEngine *getEngine(Value *v);

    ExecutionEngine *engine;
    void *firstPage;
};

class Q_QML_EXPORT PersistentValue
{
public:
    PersistentValue() : val(0) {}
    PersistentValue(const PersistentValue &other);
    PersistentValue &operator=(const PersistentValue &other);
    PersistentValue &operator=(const WeakValue &other);
    PersistentValue &operator=(Object *object);
    ~PersistentValue();

    PersistentValue(ExecutionEngine *engine, const Value &value);
    PersistentValue(ExecutionEngine *engine, ReturnedValue value);
    PersistentValue(ExecutionEngine *engine, Object *object);

    void set(ExecutionEngine *engine, const Value &value);
    void set(ExecutionEngine *engine, ReturnedValue value);
    void set(ExecutionEngine *engine, Heap::Base *obj);

    ReturnedValue value() const {
        return (val ? val->asReturnedValue() : Encode::undefined());
    }
    Value *valueRef() const {
        return val;
    }
    Managed *asManaged() const {
        if (!val)
            return 0;
        return val->asManaged();
    }

    ExecutionEngine *engine() const {
        if (!val)
            return 0;
        return PersistentValueStorage::getEngine(val);
    }

    bool isUndefined() const { return !val || val->isUndefined(); }
    bool isNullOrUndefined() const { return !val || val->isNullOrUndefined(); }
    void clear() {
        PersistentValueStorage::free(val);
        val = 0;
    }
    bool isEmpty() { return !val; }

private:
    Value *val;
};

class Q_QML_EXPORT WeakValue
{
public:
    WeakValue() : val(0) {}
    WeakValue(const WeakValue &other);
    WeakValue &operator=(const WeakValue &other);
    ~WeakValue();

    void set(ExecutionEngine *engine, const Value &value);
    void set(ExecutionEngine *engine, ReturnedValue value);
    void set(ExecutionEngine *engine, Heap::Base *obj);

    ReturnedValue value() const {
        return (val ? val->asReturnedValue() : Encode::undefined());
    }
    Value *valueRef() const {
        return val;
    }
    Managed *asManaged() const {
        if (!val)
            return 0;
        return val->asManaged();
    }

    ExecutionEngine *engine() const {
        if (!val)
            return 0;
        return PersistentValueStorage::getEngine(val);
    }

    bool isUndefined() const { return !val || val->isUndefined(); }
    bool isNullOrUndefined() const { return !val || val->isNullOrUndefined(); }
    void clear() {
        PersistentValueStorage::free(val);
        val = 0;
    }

    void markOnce(ExecutionEngine *e);

private:
    Value *val;
};

} // namespace QV4

QT_END_NAMESPACE

#endif
