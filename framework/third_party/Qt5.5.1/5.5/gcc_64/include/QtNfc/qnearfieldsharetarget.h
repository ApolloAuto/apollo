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


#ifndef QNEARFIELDSHARETARGET_H
#define QNEARFIELDSHARETARGET_H

#include <QtCore/QObject>
#include <QtCore/QFileInfo>
#include <QtNfc/QNdefMessage>
#include <QtNfc/QNearFieldShareManager>

QT_BEGIN_NAMESPACE

class QNearFieldShareTargetPrivate;

class Q_NFC_EXPORT QNearFieldShareTarget : public QObject
{
    Q_OBJECT

public:
    ~QNearFieldShareTarget();

    QNearFieldShareManager::ShareModes shareModes() const;
    bool share(const QNdefMessage &message);
    bool share(const QList<QFileInfo> &files);
    void cancel();
    bool isShareInProgress() const;
    QNearFieldShareManager::ShareError shareError() const;

Q_SIGNALS:
    void error(QNearFieldShareManager::ShareError error);
    void shareFinished();

private:
    explicit QNearFieldShareTarget(QNearFieldShareManager::ShareModes modes, QObject *parent = 0);

    QNearFieldShareTargetPrivate *d_ptr;
    Q_DECLARE_PRIVATE(QNearFieldShareTarget)
    Q_DISABLE_COPY(QNearFieldShareTarget)

    friend class QNearFieldShareManagerPrivateImpl;
    friend class QNearFieldShareTargetPrivateImpl;
};

QT_END_NAMESPACE

#endif /* QNEARFIELDSHARETARGET_H */
