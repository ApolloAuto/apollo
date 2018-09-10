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

#ifndef QQMLINFO_H
#define QQMLINFO_H

#include <QtCore/qdebug.h>
#include <QtCore/qurl.h>
#include <QtQml/qqmlerror.h>

QT_BEGIN_NAMESPACE

class QQmlInfo;

namespace QtQml {
    // declared in namespace to avoid symbol conflicts with QtDeclarative
    Q_QML_EXPORT QQmlInfo qmlInfo(const QObject *me);
    Q_QML_EXPORT QQmlInfo qmlInfo(const QObject *me, const QQmlError &error);
    Q_QML_EXPORT QQmlInfo qmlInfo(const QObject *me, const QList<QQmlError> &errors);
}
using namespace QtQml;

class QQmlInfoPrivate;
class Q_QML_EXPORT QQmlInfo : public QDebug
{
public:
    QQmlInfo(const QQmlInfo &);
    ~QQmlInfo();

    inline QQmlInfo &operator<<(QChar t) { QDebug::operator<<(t); return *this; }
    inline QQmlInfo &operator<<(bool t) { QDebug::operator<<(t); return *this; }
    inline QQmlInfo &operator<<(char t) { QDebug::operator<<(t); return *this; }
    inline QQmlInfo &operator<<(signed short t) { QDebug::operator<<(t); return *this; }
    inline QQmlInfo &operator<<(unsigned short t) { QDebug::operator<<(t); return *this; }
    inline QQmlInfo &operator<<(signed int t) { QDebug::operator<<(t); return *this; }
    inline QQmlInfo &operator<<(unsigned int t) { QDebug::operator<<(t); return *this; }
    inline QQmlInfo &operator<<(signed long t) { QDebug::operator<<(t); return *this; }
    inline QQmlInfo &operator<<(unsigned long t) { QDebug::operator<<(t); return *this; }
    inline QQmlInfo &operator<<(qint64 t) { QDebug::operator<<(t); return *this; }
    inline QQmlInfo &operator<<(quint64 t) { QDebug::operator<<(t); return *this; }
    inline QQmlInfo &operator<<(float t) { QDebug::operator<<(t); return *this; }
    inline QQmlInfo &operator<<(double t) { QDebug::operator<<(t); return *this; }
    inline QQmlInfo &operator<<(const char* t) { QDebug::operator<<(t); return *this; }
    inline QQmlInfo &operator<<(const QString & t) { QDebug::operator<<(t.toLocal8Bit().constData()); return *this; }
    inline QQmlInfo &operator<<(const QStringRef & t) { return operator<<(t.toString()); }
    inline QQmlInfo &operator<<(const QLatin1String &t) { QDebug::operator<<(t.latin1()); return *this; }
    inline QQmlInfo &operator<<(const QByteArray & t) { QDebug::operator<<(t); return *this; }
    inline QQmlInfo &operator<<(const void * t) { QDebug::operator<<(t); return *this; }
    inline QQmlInfo &operator<<(QTextStreamFunction f) { QDebug::operator<<(f); return *this; }
    inline QQmlInfo &operator<<(QTextStreamManipulator m) { QDebug::operator<<(m); return *this; }
#ifndef QT_NO_DEBUG_STREAM
    inline QQmlInfo &operator<<(const QUrl &t) { static_cast<QDebug &>(*this) << t; return *this; }
#endif

private:
    friend Q_QML_EXPORT QQmlInfo QtQml::qmlInfo(const QObject *me);
    friend Q_QML_EXPORT QQmlInfo QtQml::qmlInfo(const QObject *me, const QQmlError &error);
    friend Q_QML_EXPORT QQmlInfo QtQml::qmlInfo(const QObject *me, const QList<QQmlError> &errors);

    QQmlInfo(QQmlInfoPrivate *);
    QQmlInfoPrivate *d;
};

QT_END_NAMESPACE

#endif // QQMLINFO_H
