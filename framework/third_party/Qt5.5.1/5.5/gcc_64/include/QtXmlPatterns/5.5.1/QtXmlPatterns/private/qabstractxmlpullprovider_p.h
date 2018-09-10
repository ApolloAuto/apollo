/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the QtXmlPatterns module of the Qt Toolkit.
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

//
//  W A R N I N G
//  -------------
//
// This file is not part of the Qt API.  It exists purely as an
// implementation detail.  This header file may change from version to
// version without notice, or even be removed.
//
// We mean it.

#ifndef QABSTRACTXMLPULLPROVIDER_H
#define QABSTRACTXMLPULLPROVIDER_H

#include <QtCore/QtGlobal>

QT_BEGIN_NAMESPACE

class QXmlItem;
class QXmlName;
class QString;
class QVariant;
template<typename Key, typename Value> class QHash;

namespace QPatternist
{
    class AbstractXmlPullProviderPrivate;

    class AbstractXmlPullProvider
    {
    public:
        AbstractXmlPullProvider();
        virtual ~AbstractXmlPullProvider();

        enum Event
        {
            StartOfInput            = 1,
            AtomicValue             = 1 << 1,
            StartDocument           = 1 << 2,
            EndDocument             = 1 << 3,
            StartElement            = 1 << 4,
            EndElement              = 1 << 5,
            Text                    = 1 << 6,
            ProcessingInstruction   = 1 << 7,
            Comment                 = 1 << 8,
            Attribute               = 1 << 9,
            Namespace               = 1 << 10,
            EndOfInput              = 1 << 11
        };

        virtual Event next() = 0;
        virtual Event current() const = 0;
        virtual QXmlName name() const = 0;
        virtual QVariant atomicValue() const = 0;
        virtual QString stringValue() const = 0;

        virtual QHash<QXmlName, QString> attributes() = 0;
        virtual QHash<QXmlName, QXmlItem> attributeItems() = 0;

        /* *** The functions below are internal. */
    private:
        Q_DISABLE_COPY(AbstractXmlPullProvider)
    };
}

QT_END_NAMESPACE

#endif
