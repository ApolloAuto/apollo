/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Copyright (C) 2014 by Southwest Research Institute (R)
** Contact: http://www.qt.io/licensing/
**
** This file is part of the QtCore module of the Qt Toolkit.
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

#include <QtCore/qlist.h>

#ifndef QBYTEARRAYLIST_H
#define QBYTEARRAYLIST_H

#include <QtCore/qbytearray.h>

QT_BEGIN_NAMESPACE

typedef QListIterator<QByteArray> QByteArrayListIterator;
typedef QMutableListIterator<QByteArray> QMutableByteArrayListIterator;
typedef QList<QByteArray> QByteArrayList;

namespace QtPrivate {
    QByteArray Q_CORE_EXPORT QByteArrayList_join(const QByteArrayList *that, const char *separator, int separatorLength);
}

#ifdef Q_QDOC
class QByteArrayList : public QList<QByteArray>
#else
template <> struct QListSpecialMethods<QByteArray>
#endif
{
#ifndef Q_QDOC
protected:
    ~QListSpecialMethods() {}
#endif
public:
    inline QByteArray join() const
    { return QtPrivate::QByteArrayList_join(self(), 0, 0); }
    inline QByteArray join(const QByteArray &sep) const
    { return QtPrivate::QByteArrayList_join(self(), sep.constData(), sep.size()); }
    inline QByteArray join(char sep) const
    { return QtPrivate::QByteArrayList_join(self(), &sep, 1); }

private:
    typedef QList<QByteArray> Self;
    Self *self() { return static_cast<Self *>(this); }
    const Self *self() const { return static_cast<const Self *>(this); }
};

QT_END_NAMESPACE

#endif // QBYTEARRAYLIST_H
