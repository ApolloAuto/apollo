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

#ifndef QQMLFILE_H
#define QQMLFILE_H

#include <QtQml/qtqmlglobal.h>

QT_BEGIN_NAMESPACE

class QUrl;
class QString;
class QObject;
class QQmlEngine;
class QQmlFilePrivate;

class Q_QML_EXPORT QQmlFile
{
public:
    QQmlFile();
    QQmlFile(QQmlEngine *, const QUrl &);
    QQmlFile(QQmlEngine *, const QString &);
    ~QQmlFile();

    enum Status { Null, Ready, Error, Loading };

    bool isNull() const;
    bool isReady() const;
    bool isError() const;
    bool isLoading() const;

    QUrl url() const;

    Status status() const;
    QString error() const;

    qint64 size() const;
    const char *data() const;
    QByteArray dataByteArray() const;

    void load(QQmlEngine *, const QUrl &);
    void load(QQmlEngine *, const QString &);

    void clear();
    void clear(QObject *);

    bool connectFinished(QObject *, const char *);
    bool connectFinished(QObject *, int);
    bool connectDownloadProgress(QObject *, const char *);
    bool connectDownloadProgress(QObject *, int);

    static bool isSynchronous(const QString &url);
    static bool isSynchronous(const QUrl &url);

    static bool isLocalFile(const QString &url);
    static bool isLocalFile(const QUrl &url);

    static QString urlToLocalFileOrQrc(const QString &);
    static QString urlToLocalFileOrQrc(const QUrl &);
private:
    Q_DISABLE_COPY(QQmlFile)
    QQmlFilePrivate *d;
};

QT_END_NAMESPACE

#endif // QQMLFILE_H
