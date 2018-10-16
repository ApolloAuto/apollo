/****************************************************************************
**
** Copyright (C) 2014 Ivan Komissarov <ABBAPOH@gmail.com>
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

#ifndef QSTORAGEINFO_H
#define QSTORAGEINFO_H

#include <QtCore/qbytearray.h>
#include <QtCore/qdir.h>
#include <QtCore/qlist.h>
#include <QtCore/qmetatype.h>
#include <QtCore/qstring.h>
#include <QtCore/qshareddata.h>

QT_BEGIN_NAMESPACE

class QStorageInfoPrivate;
class Q_CORE_EXPORT QStorageInfo
{
public:
    QStorageInfo();
    explicit QStorageInfo(const QString &path);
    explicit QStorageInfo(const QDir &dir);
    QStorageInfo(const QStorageInfo &other);
    ~QStorageInfo();

    QStorageInfo &operator=(const QStorageInfo &other);
#ifdef Q_COMPILER_RVALUE_REFS
    inline QStorageInfo &operator=(QStorageInfo &&other)
    { qSwap(d, other.d); return *this; }
#endif

    inline void swap(QStorageInfo &other)
    { qSwap(d, other.d); }

    void setPath(const QString &path);

    QString rootPath() const;
    QByteArray device() const;
    QByteArray fileSystemType() const;
    QString name() const;
    QString displayName() const;

    qint64 bytesTotal() const;
    qint64 bytesFree() const;
    qint64 bytesAvailable() const;

    inline bool isRoot() const;
    bool isReadOnly() const;
    bool isReady() const;
    bool isValid() const;

    void refresh();

    static QList<QStorageInfo> mountedVolumes();
    static QStorageInfo root();

private:
    friend class QStorageInfoPrivate;
    friend bool operator==(const QStorageInfo &first, const QStorageInfo &second);
    QExplicitlySharedDataPointer<QStorageInfoPrivate> d;
};

inline bool operator==(const QStorageInfo &first, const QStorageInfo &second)
{
    if (first.d == second.d)
        return true;
    return first.device() == second.device();
}

inline bool operator!=(const QStorageInfo &first, const QStorageInfo &second)
{
    return !(first == second);
}

inline bool QStorageInfo::isRoot() const
{ return *this == QStorageInfo::root(); }

Q_DECLARE_SHARED(QStorageInfo)

QT_END_NAMESPACE

Q_DECLARE_METATYPE(QStorageInfo)

#endif // QSTORAGEINFO_H
