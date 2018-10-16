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

#ifndef QXMLSERIALIZER_P_H
#define QXMLSERIALIZER_P_H

#include <QtCore/QIODevice>
#include <QtCore/QStack>
#include <QtCore/QTextCodec>
#include <QtXmlPatterns/QXmlQuery>
#include <QtXmlPatterns/QXmlNamePool>
#include <QtXmlPatterns/QXmlSerializer>

#include <private/qnamepool_p.h>
#include <private/qabstractxmlreceiver_p.h>

QT_BEGIN_NAMESPACE

class QXmlSerializerPrivate : public QAbstractXmlReceiverPrivate
{
public:
    QXmlSerializerPrivate(const QXmlQuery &q,
                          QIODevice *outputDevice);

    QStack<QPair<QXmlName, bool> >      hasClosedElement;
    bool                                isPreviousAtomic;
    QXmlSerializer::State               state;
    const QPatternist::NamePool::Ptr    np;

    /**
     * This member worries me a bit. We never use it but nevertheless
     * it is pushed and pops linear to startElement() and endElement().
     * An optimization would be to at least merge it with hasClosedElement,
     * but even better to push it on demand. That is, namespaceBinding()
     * pushes it up to the tree depth first when it is needed.
     */
    QStack<QVector<QXmlName> >          namespaces;

    QIODevice *                         device;
    const QTextCodec *                  codec;
    QTextCodec::ConverterState          converterState;
    /**
     * Name cache. Since encoding QStrings are rather expensive
     * operations to do, and we on top of that would have to do
     * it each time a name appears, we here map names to their
     * encoded equivalents.
     *
     * This means that when writing out large documents, the serialization
     * of names after a while is reduced to a hash lookup and passing an
     * existing byte array.
     *
     * We use QXmlName::Code as key as opposed to merely QName, because the
     * prefix is of significance.
     */
    QHash<QXmlName::Code, QByteArray>   nameCache;
    const QXmlQuery                     query;

    inline void write(const char c);

private:
    enum Constants
    {
        EstimatedTreeDepth = 10,

        /**
         * We use a high count to avoid rehashing. We can afford it since we
         * only allocate one hash for this.
         */
        EstimatedNameCount = 60
    };
};

void QXmlSerializerPrivate::write(const char c)
{
    device->putChar(c);
}
QT_END_NAMESPACE

#endif
