/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the QtNetwork module of the Qt Toolkit.
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


#ifndef QSSLKEY_OPENSSL_P_H
#define QSSLKEY_OPENSSL_P_H

#include "qsslkey.h"

//
//  W A R N I N G
//  -------------
//
// This file is not part of the Qt API.  It exists for the convenience
// of qsslcertificate.cpp.  This header file may change from version to version
// without notice, or even be removed.
//
// We mean it.
//

#include "qsslsocket_p.h" // includes wincrypt.h

#ifndef QT_NO_OPENSSL
#include <openssl/rsa.h>
#include <openssl/dsa.h>
#endif

QT_BEGIN_NAMESPACE

class QSslKeyPrivate
{
public:
    inline QSslKeyPrivate()
        : algorithm(QSsl::Opaque)
        , opaque(0)
    {
        clear(false);
    }

    inline ~QSslKeyPrivate()
    { clear(); }

    void clear(bool deep = true);

#ifndef QT_NO_OPENSSL
    bool fromEVP_PKEY(EVP_PKEY *pkey);
#endif
    void decodeDer(const QByteArray &der, bool deepClear = true);
    void decodePem(const QByteArray &pem, const QByteArray &passPhrase,
                   bool deepClear = true);
    QByteArray pemHeader() const;
    QByteArray pemFooter() const;
    QByteArray pemFromDer(const QByteArray &der, const QMap<QByteArray, QByteArray> &headers) const;
    QByteArray derFromPem(const QByteArray &pem, QMap<QByteArray, QByteArray> *headers) const;

    int length() const;
    QByteArray toPem(const QByteArray &passPhrase) const;
    Qt::HANDLE handle() const;

    bool isNull;
    QSsl::KeyType type;
    QSsl::KeyAlgorithm algorithm;
#ifndef QT_NO_OPENSSL
    union {
        EVP_PKEY *opaque;
        RSA *rsa;
        DSA *dsa;
#ifndef OPENSSL_NO_EC
        EC_KEY *ec;
#endif
    };
#else
    enum Cipher {
        DesCbc,
        DesEde3Cbc,
        Rc2Cbc
    };

    Q_AUTOTEST_EXPORT static QByteArray decrypt(Cipher cipher, const QByteArray &data, const QByteArray &key, const QByteArray &iv);
    Q_AUTOTEST_EXPORT static QByteArray encrypt(Cipher cipher, const QByteArray &data, const QByteArray &key, const QByteArray &iv);

    Qt::HANDLE opaque;
    QByteArray derData;
    int keyLength;
#endif

    QAtomicInt ref;

private:
    Q_DISABLE_COPY(QSslKeyPrivate)
};

QT_END_NAMESPACE

#endif // QSSLKEY_OPENSSL_P_H
