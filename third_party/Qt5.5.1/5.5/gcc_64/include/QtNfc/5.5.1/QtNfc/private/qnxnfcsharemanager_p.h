/***************************************************************************
 **
 ** Copyright (C) 2013 BlackBerry Limited. All rights reserved.
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

#ifndef QNXNFCSHAREMANAGER_P_H
#define QNXNFCSHAREMANAGER_P_H

//
//  W A R N I N G
//  -------------
//
// This file is not part of the Qt API.  It exists purely as an
// implementation detail.  This header file may change from version to
// version without notice, or even be removed.
//
// We mean it.
//

#include "qnearfieldsharemanager.h"
#include <bb/system/NfcShareManager>
#include <QFileInfo>
#include <QNdefMessage>

QT_BEGIN_NAMESPACE

class Q_DECL_EXPORT QNXNFCShareManager : public QObject
{
    Q_OBJECT
public:
    static QNXNFCShareManager *instance();

    QNXNFCShareManager();
    ~QNXNFCShareManager();

    bool shareNdef(const QNdefMessage &message);
    bool shareFiles(const QList<QFileInfo> &files);
    void cancel();

    void setShareMode(bb::system::NfcShareMode::Type type);
    bb::system::NfcShareMode::Type shareMode() const;

    void connect(QObject *obj);
    void disconnect(QObject *obj);

    void reset();

    static QNearFieldShareManager::ShareError toShareError(bb::system::NfcShareError::Type nfcShareError);
    static QNearFieldShareManager::ShareModes toShareModes(bb::system::NfcShareMode::Type nfcShareMode);

private:
    bb::system::NfcShareManager *_manager;
    static const char *RECORD_NDEF;

signals:
    void shareModeChanged(bb::system::NfcShareMode::Type);
    void error(bb::system::NfcShareError::Type);
    void finished(bb::system::NfcShareSuccess::Type);
    void targetAcquired();
    void targetCancelled();
};

QT_END_NAMESPACE

#endif /* QNXNFCSHAREMANAGER_P_H */
