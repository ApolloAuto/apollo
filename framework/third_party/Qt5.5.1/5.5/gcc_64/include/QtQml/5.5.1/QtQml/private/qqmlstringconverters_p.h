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

#ifndef QQMLSTRINGCONVERTERS_P_H
#define QQMLSTRINGCONVERTERS_P_H

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

#include <QtCore/qglobal.h>
#include <QtCore/qvariant.h>

#include <private/qtqmlglobal_p.h>

QT_BEGIN_NAMESPACE

class QPointF;
class QSizeF;
class QRectF;
class QString;
class QByteArray;

namespace QQmlStringConverters
{
    Q_QML_PRIVATE_EXPORT QVariant variantFromString(const QString &);
    Q_QML_PRIVATE_EXPORT QVariant variantFromString(const QString &, int preferredType, bool *ok = 0);

    Q_QML_PRIVATE_EXPORT QVariant colorFromString(const QString &, bool *ok = 0);
    Q_QML_PRIVATE_EXPORT unsigned rgbaFromString(const QString &, bool *ok = 0);

#ifndef QT_NO_DATESTRING
    Q_QML_PRIVATE_EXPORT QDate dateFromString(const QString &, bool *ok = 0);
    Q_QML_PRIVATE_EXPORT QTime timeFromString(const QString &, bool *ok = 0);
    Q_QML_PRIVATE_EXPORT QDateTime dateTimeFromString(const QString &, bool *ok = 0);
#endif
    Q_QML_PRIVATE_EXPORT QPointF pointFFromString(const QString &, bool *ok = 0);
    Q_QML_PRIVATE_EXPORT QSizeF sizeFFromString(const QString &, bool *ok = 0);
    Q_QML_PRIVATE_EXPORT QRectF rectFFromString(const QString &, bool *ok = 0);

    Q_QML_PRIVATE_EXPORT bool createFromString(int, const QString &, void *, size_t);
}

QT_END_NAMESPACE

#endif // QQMLSTRINGCONVERTERS_P_H
