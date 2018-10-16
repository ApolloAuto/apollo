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

#ifndef QHASHFIELD_P_H
#define QHASHFIELD_P_H

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

QT_BEGIN_NAMESPACE

// QHashField can be used for doing coarse grained set testing, in
// cases where you do not expect the set to contain the item.  For
// example where you would write:
//     QSet<QString> strings;
//     for (int ii = 0; ii < mystrings.count(); ++ii) {
//         if (strings.contains(mystrings.at(ii)))
//             qFatal("Duplication!");
//         strings.insert(mystrings);
//     }
// You may write:
//     QHashField strings;
//     for (int ii = 0; ii < mystrings.count(); ++ii) {
//         if (strings.testAndSet(qHash(mystrings.at(ii)))) {
//             // The string *might* be duplicated
//             for (int jj = 0; jj < ii; ++jj) {
//                 if (mystrings.at(ii) == mystrings.at(jj))
//                     qFatal("Duplication!");
//             }
//          }
//     }
// For small lists of things, where the hash is cheap to calculate
// and you don't expect duplication this will be much faster.
class QHashField {
public:
    inline QHashField();

    inline void clear();

    inline bool test(quint32 hash);
    inline bool testAndSet(quint32 hash);
private:
    quint32 m_field;
};

QHashField::QHashField()
: m_field(0)
{
}

void QHashField::clear()
{
    m_field = 0;
}

bool QHashField::test(quint32 hash)
{
    return m_field & (1 << (hash % 31));
}

bool QHashField::testAndSet(quint32 hash)
{
    quint32 mask = 1 << (hash % 31);
    bool rv = m_field & mask;
    m_field |= mask;
    return rv;
}

QT_END_NAMESPACE

#endif // QHASHFIELD_P_H
