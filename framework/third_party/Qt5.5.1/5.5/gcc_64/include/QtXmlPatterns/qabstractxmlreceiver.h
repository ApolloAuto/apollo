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

#ifndef QABSTRACTXMLRECEIVER_H
#define QABSTRACTXMLRECEIVER_H

#include <QtCore/QVariant>
#include <QtCore/QScopedPointer>
#include <QtXmlPatterns/QXmlNodeModelIndex>

QT_BEGIN_NAMESPACE


class QAbstractXmlReceiverPrivate;
class QXmlName;

namespace QPatternist
{
    class Item;
}

class Q_XMLPATTERNS_EXPORT QAbstractXmlReceiver
{
public:
    QAbstractXmlReceiver();

    virtual ~QAbstractXmlReceiver();

    virtual void startElement(const QXmlName &name) = 0;
    virtual void endElement() = 0;
    virtual void attribute(const QXmlName &name,
                           const QStringRef &value) = 0;
    virtual void comment(const QString &value) = 0;
    virtual void characters(const QStringRef &value) = 0;
    virtual void startDocument() = 0;
    virtual void endDocument() = 0;

    virtual void processingInstruction(const QXmlName &target,
                                       const QString &value) = 0;

    virtual void atomicValue(const QVariant &value) = 0;
    virtual void namespaceBinding(const QXmlName &name) = 0;
    virtual void startOfSequence() = 0;
    virtual void endOfSequence() = 0;

    /* The members below are internal, not part of the public API, and
     * unsupported. Using them leads to undefined behavior. */
    virtual void whitespaceOnly(const QStringRef &value);
    virtual void item(const QPatternist::Item &item);

protected:
    QAbstractXmlReceiver(QAbstractXmlReceiverPrivate *d);
    QScopedPointer<QAbstractXmlReceiverPrivate> d_ptr;

    void sendAsNode(const QPatternist::Item &outputItem);
private:
    template<const QXmlNodeModelIndex::Axis axis>
    void sendFromAxis(const QXmlNodeModelIndex &node);
    Q_DISABLE_COPY(QAbstractXmlReceiver)
};

QT_END_NAMESPACE

#endif
