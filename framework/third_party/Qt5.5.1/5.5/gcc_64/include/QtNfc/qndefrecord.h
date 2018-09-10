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

#ifndef QNDEFRECORD_H
#define QNDEFRECORD_H

#include <QtCore/QSharedDataPointer>
#include <QtCore/QByteArray>
#include <QtNfc/qnfcglobal.h>

QT_BEGIN_NAMESPACE

class QNdefRecordPrivate;

class Q_NFC_EXPORT QNdefRecord
{
public:
    enum TypeNameFormat {
        Empty = 0x00,
        NfcRtd = 0x01,
        Mime = 0x02,
        Uri = 0x03,
        ExternalRtd = 0x04,
        Unknown = 0x05
    };

    QNdefRecord();
    ~QNdefRecord();

    QNdefRecord(const QNdefRecord &other);
    QNdefRecord &operator=(const QNdefRecord &other);

    void setTypeNameFormat(TypeNameFormat typeNameFormat);
    TypeNameFormat typeNameFormat() const;

    void setType(const QByteArray &type);
    QByteArray type() const;

    void setId(const QByteArray &id);
    QByteArray id() const;

    void setPayload(const QByteArray &payload);
    QByteArray payload() const;

    bool isEmpty() const;

    template <typename T>
    inline bool isRecordType() const
    {
        T dummy;
        return (typeNameFormat() == dummy.typeNameFormat() && type() == dummy.type());
    }

    bool operator==(const QNdefRecord &other) const;
    inline bool operator!=(const QNdefRecord &other) const { return !operator==(other); }

protected:
    QNdefRecord(const QNdefRecord &other, TypeNameFormat typeNameFormat, const QByteArray &type);
    QNdefRecord(const QNdefRecord &other, TypeNameFormat typeNameFormat);
    QNdefRecord(TypeNameFormat typeNameFormat, const QByteArray &type);

private:
    QSharedDataPointer<QNdefRecordPrivate> d;
};

#define Q_DECLARE_NDEF_RECORD(className, typeNameFormat, type, initialPayload) \
    className() : QNdefRecord(typeNameFormat, type) { setPayload(initialPayload); } \
    className(const QNdefRecord &other) : QNdefRecord(other, typeNameFormat, type) { }

#define Q_DECLARE_ISRECORDTYPE_FOR_NDEF_RECORD(className, typeNameFormat_, type_) \
    QT_BEGIN_NAMESPACE \
    template<> inline bool QNdefRecord::isRecordType<className>() const\
    { \
        return (typeNameFormat() == typeNameFormat_ && type() == type_); \
    } \
    QT_END_NAMESPACE

Q_NFC_EXPORT uint qHash(const QNdefRecord &key);

QT_END_NAMESPACE

#endif // QNDEFRECORD_H
