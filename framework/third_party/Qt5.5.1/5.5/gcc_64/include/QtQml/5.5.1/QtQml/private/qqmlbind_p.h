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

#ifndef QQMLBIND_H
#define QQMLBIND_H

#include <qqml.h>

#include <QtCore/qobject.h>

QT_BEGIN_NAMESPACE

class QQmlBindPrivate;
class Q_AUTOTEST_EXPORT QQmlBind : public QObject, public QQmlPropertyValueSource, public QQmlParserStatus
{
    Q_OBJECT
    Q_DECLARE_PRIVATE(QQmlBind)
    Q_INTERFACES(QQmlParserStatus)
    Q_INTERFACES(QQmlPropertyValueSource)
    Q_PROPERTY(QObject *target READ object WRITE setObject)
    Q_PROPERTY(QString property READ property WRITE setProperty)
    Q_PROPERTY(QVariant value READ value WRITE setValue)
    Q_PROPERTY(bool when READ when WRITE setWhen)

public:
    QQmlBind(QObject *parent=0);
    ~QQmlBind();

    bool when() const;
    void setWhen(bool);

    QObject *object();
    void setObject(QObject *);

    QString property() const;
    void setProperty(const QString &);

    QVariant value() const;
    void setValue(const QVariant &);

protected:
    virtual void setTarget(const QQmlProperty &);
    virtual void classBegin();
    virtual void componentComplete();

private:
    void eval();
};

QT_END_NAMESPACE

QML_DECLARE_TYPE(QQmlBind)

#endif
