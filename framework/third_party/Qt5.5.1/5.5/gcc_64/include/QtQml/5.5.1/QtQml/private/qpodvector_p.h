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

#ifndef QPODVECTOR_P_H
#define QPODVECTOR_P_H

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

#include <QtCore/qglobal.h>
#include <QDebug>

QT_BEGIN_NAMESPACE

template<class T, int Increment>
class QPODVector
{
public:
    QPODVector()
    : m_count(0), m_capacity(0), m_data(0) {}
    ~QPODVector() { if (m_data) ::free(m_data); }

    const T &at(int idx) const {
        return m_data[idx];
    }

    T &operator[](int idx) {
        return m_data[idx];
    }

    void clear() {
        m_count = 0;
    }

    void prepend(const T &v) {
        insert(0, v);
    }

    void append(const T &v) {
        insert(m_count, v);
    }

    void insert(int idx, const T &v) {
        if (m_count == m_capacity) {
            m_capacity += Increment;
            m_data = (T *)realloc(m_data, m_capacity * sizeof(T));
        }
        int moveCount = m_count - idx;
        if (moveCount)
            ::memmove(m_data + idx + 1, m_data + idx, moveCount * sizeof(T));
        m_count++;
        m_data[idx] = v;
    }

    void reserve(int count) {
        if (count >= m_capacity) {
            m_capacity = (count + (Increment-1)) & (0xFFFFFFFF - Increment + 1);
            m_data = (T *)realloc(m_data, m_capacity * sizeof(T));
        }
    }

    void insertBlank(int idx, int count) {
        int newSize = m_count + count;
        reserve(newSize);
        int moveCount = m_count - idx;
        if (moveCount)
            ::memmove(m_data + idx + count,  m_data + idx,
                      moveCount * sizeof(T));
        m_count = newSize;
    }

    void remove(int idx, int count = 1) {
        int moveCount = m_count - (idx + count);
        if (moveCount)
            ::memmove(m_data + idx, m_data + idx + count,
                      moveCount * sizeof(T));
        m_count -= count;
    }

    void removeOne(const T &v) {
        int idx = 0;
        while (idx < m_count) {
            if (m_data[idx] == v) {
                remove(idx);
                return;
            }
            ++idx;
        }
    }

    int find(const T &v) {
        for (int idx = 0; idx < m_count; ++idx)
            if (m_data[idx] == v)
                return idx;
        return -1;
    }

    bool contains(const T &v) {
        return find(v) != -1;
    }

    int count() const {
        return m_count;
    }

    void copyAndClear(QPODVector<T,Increment> &other) {
        if (other.m_data) ::free(other.m_data);
        other.m_count = m_count;
        other.m_capacity = m_capacity;
        other.m_data = m_data;
        m_count = 0;
        m_capacity = 0;
        m_data = 0;
    }

    QPODVector<T,Increment> &operator<<(const T &v) { append(v); return *this; }
private:
    QPODVector(const QPODVector &);
    QPODVector &operator=(const QPODVector &);
    int m_count;
    int m_capacity;
    T *m_data;
};

QT_END_NAMESPACE

#endif
