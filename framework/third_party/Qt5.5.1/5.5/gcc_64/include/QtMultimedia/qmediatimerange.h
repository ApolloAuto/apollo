/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the Qt Toolkit.
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

#ifndef QMEDIATIMERANGE_H
#define QMEDIATIMERANGE_H

#include <QtMultimedia/qtmultimediadefs.h>
#include <QtMultimedia/qmultimedia.h>
#include <QtCore/qshareddata.h>

QT_BEGIN_NAMESPACE


class QMediaTimeRangePrivate;

class Q_MULTIMEDIA_EXPORT QMediaTimeInterval
{
public:
    QMediaTimeInterval();
    QMediaTimeInterval(qint64 start, qint64 end);
    QMediaTimeInterval(const QMediaTimeInterval&);

    qint64 start() const;
    qint64 end() const;

    bool contains(qint64 time) const;

    bool isNormal() const;
    QMediaTimeInterval normalized() const;
    QMediaTimeInterval translated(qint64 offset) const;

private:
    friend class QMediaTimeRangePrivate;
    friend class QMediaTimeRange;

    qint64 s;
    qint64 e;
};

Q_MULTIMEDIA_EXPORT bool operator==(const QMediaTimeInterval&, const QMediaTimeInterval&);
Q_MULTIMEDIA_EXPORT bool operator!=(const QMediaTimeInterval&, const QMediaTimeInterval&);

class Q_MULTIMEDIA_EXPORT QMediaTimeRange
{
public:

    QMediaTimeRange();
    QMediaTimeRange(qint64 start, qint64 end);
    QMediaTimeRange(const QMediaTimeInterval&);
    QMediaTimeRange(const QMediaTimeRange &range);
    ~QMediaTimeRange();

    QMediaTimeRange &operator=(const QMediaTimeRange&);
    QMediaTimeRange &operator=(const QMediaTimeInterval&);

    qint64 earliestTime() const;
    qint64 latestTime() const;

    QList<QMediaTimeInterval> intervals() const;
    bool isEmpty() const;
    bool isContinuous() const;

    bool contains(qint64 time) const;

    void addInterval(qint64 start, qint64 end);
    void addInterval(const QMediaTimeInterval &interval);
    void addTimeRange(const QMediaTimeRange&);

    void removeInterval(qint64 start, qint64 end);
    void removeInterval(const QMediaTimeInterval &interval);
    void removeTimeRange(const QMediaTimeRange&);

    QMediaTimeRange& operator+=(const QMediaTimeRange&);
    QMediaTimeRange& operator+=(const QMediaTimeInterval&);
    QMediaTimeRange& operator-=(const QMediaTimeRange&);
    QMediaTimeRange& operator-=(const QMediaTimeInterval&);

    void clear();

private:
    QSharedDataPointer<QMediaTimeRangePrivate> d;
};

Q_MULTIMEDIA_EXPORT bool operator==(const QMediaTimeRange&, const QMediaTimeRange&);
Q_MULTIMEDIA_EXPORT bool operator!=(const QMediaTimeRange&, const QMediaTimeRange&);
Q_MULTIMEDIA_EXPORT QMediaTimeRange operator+(const QMediaTimeRange&, const QMediaTimeRange&);
Q_MULTIMEDIA_EXPORT QMediaTimeRange operator-(const QMediaTimeRange&, const QMediaTimeRange&);

#ifndef QT_NO_DEBUG_STREAM
Q_MULTIMEDIA_EXPORT QDebug operator<<(QDebug, const QMediaTimeRange &);
#endif

QT_END_NAMESPACE


#endif  // QMEDIATIMERANGE_H
