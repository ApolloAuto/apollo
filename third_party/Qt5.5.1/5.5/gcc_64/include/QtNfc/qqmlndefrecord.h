/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the QtNfc module of the Qt Toolkit.
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

#ifndef QQMLNDEFRECORD_H
#define QQMLNDEFRECORD_H

#include <QtCore/QObject>
#include <QtCore/QMetaType>
#include <QtNfc/QNdefRecord>

QT_BEGIN_NAMESPACE

class QQmlNdefRecordPrivate;

class Q_NFC_EXPORT QQmlNdefRecord : public QObject
{
    Q_OBJECT

    Q_DECLARE_PRIVATE(QQmlNdefRecord)

    Q_PROPERTY(QString type READ type WRITE setType NOTIFY typeChanged)
    Q_PROPERTY(TypeNameFormat typeNameFormat READ typeNameFormat WRITE setTypeNameFormat NOTIFY typeNameFormatChanged)
    Q_PROPERTY(QNdefRecord record READ record WRITE setRecord NOTIFY recordChanged)

public:
    enum TypeNameFormat {
        Empty = QNdefRecord::Empty,
        NfcRtd = QNdefRecord::NfcRtd,
        Mime = QNdefRecord::Mime,
        Uri = QNdefRecord::Uri,
        ExternalRtd = QNdefRecord::ExternalRtd,
        Unknown = QNdefRecord::Unknown
    };
    Q_ENUM(TypeNameFormat)

    explicit QQmlNdefRecord(QObject *parent = 0);
    explicit QQmlNdefRecord(const QNdefRecord &record, QObject *parent = 0);

    QString type() const;
    void setType(const QString &t);

    void setTypeNameFormat(TypeNameFormat typeNameFormat);
    TypeNameFormat typeNameFormat() const;

    QNdefRecord record() const;
    void setRecord(const QNdefRecord &record);

Q_SIGNALS:
    void typeChanged();
    void typeNameFormatChanged();
    void recordChanged();

private:
    QQmlNdefRecordPrivate *d_ptr;
};

void Q_NFC_EXPORT qRegisterNdefRecordTypeHelper(const QMetaObject *metaObject,
                                                         QNdefRecord::TypeNameFormat typeNameFormat,
                                                         const QByteArray &type);

Q_NFC_EXPORT QQmlNdefRecord *qNewDeclarativeNdefRecordForNdefRecord(const QNdefRecord &record);

template<typename T>
bool qRegisterNdefRecordType(QNdefRecord::TypeNameFormat typeNameFormat, const QByteArray &type)
{
    qRegisterNdefRecordTypeHelper(&T::staticMetaObject, typeNameFormat, type);
    return true;
}

#define Q_DECLARE_NDEFRECORD(className, typeNameFormat, type) \
static bool _q_##className##_registered = qRegisterNdefRecordType<className>(typeNameFormat, type);

QT_END_NAMESPACE

#endif // QQMLNDEFRECORD_H
