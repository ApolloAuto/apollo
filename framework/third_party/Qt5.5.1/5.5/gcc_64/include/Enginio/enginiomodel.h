/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the QtEnginio module of the Qt Toolkit.
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

#ifndef ENGINIOMODEL_H
#define ENGINIOMODEL_H

#include <QtCore/qjsonobject.h>
#include <QtCore/qscopedpointer.h>

#include <Enginio/enginioclient.h>
#include <Enginio/enginiobasemodel.h>

QT_BEGIN_NAMESPACE

class EnginioModelPrivate;
class ENGINIOCLIENT_EXPORT EnginioModel
#ifdef Q_QDOC
        : public QAbstractListModel
#else
        : public EnginioBaseModel
#endif
{
    Q_OBJECT
    Q_ENUMS(Enginio::Operation) // TODO remove me QTBUG-33577
    Q_PROPERTY(Enginio::Operation operation READ operation WRITE setOperation NOTIFY operationChanged)
    Q_PROPERTY(EnginioClient *client READ client WRITE setClient NOTIFY clientChanged)
    Q_PROPERTY(QJsonObject query READ query WRITE setQuery NOTIFY queryChanged)

public:
    explicit EnginioModel(QObject *parent = 0);
    ~EnginioModel();

    EnginioClient *client() const Q_REQUIRED_RESULT;
    void setClient(const EnginioClient *client);

    QJsonObject query() Q_REQUIRED_RESULT;
    void setQuery(const QJsonObject &query);

    Enginio::Operation operation() const Q_REQUIRED_RESULT;
    void setOperation(Enginio::Operation operation);

    Q_INVOKABLE EnginioReply *append(const QJsonObject &value);
    Q_INVOKABLE EnginioReply *remove(int row);
    Q_INVOKABLE EnginioReply *setData(int row, const QVariant &value, const QString &role);
    Q_INVOKABLE EnginioReply *setData(int row, const QJsonObject &value);
    using EnginioBaseModel::setData;

    Q_INVOKABLE EnginioReply *reload();

Q_SIGNALS:
    void queryChanged(const QJsonObject &query);
    void clientChanged(EnginioClient *client);
    void operationChanged(Enginio::Operation operation);

private:
    Q_DISABLE_COPY(EnginioModel)
    Q_DECLARE_PRIVATE(EnginioModel)
    friend class EnginioBaseModelPrivate;
};

QT_END_NAMESPACE

#endif // ENGINIOMODEL_H
