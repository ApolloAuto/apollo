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

#ifndef QXMLSERIALIZER_H
#define QXMLSERIALIZER_H

#include <QtXmlPatterns/QAbstractXmlReceiver>

QT_BEGIN_NAMESPACE


class QIODevice;
class QTextCodec;
class QXmlQuery;
class QXmlSerializerPrivate;

class Q_XMLPATTERNS_EXPORT QXmlSerializer : public QAbstractXmlReceiver
{
public:
    QXmlSerializer(const QXmlQuery &query,
                   QIODevice *outputDevice);

    virtual void namespaceBinding(const QXmlName &nb);

    virtual void characters(const QStringRef &value);
    virtual void comment(const QString &value);

    virtual void startElement(const QXmlName &name);

    virtual void endElement();

    virtual void attribute(const QXmlName &name,
                           const QStringRef &value);

    virtual void processingInstruction(const QXmlName &name,
                                       const QString &value);

    virtual void atomicValue(const QVariant &value);

    virtual void startDocument();
    virtual void endDocument();
    virtual void startOfSequence();
    virtual void endOfSequence();

    QIODevice *outputDevice() const;

    void setCodec(const QTextCodec *codec);
    const QTextCodec *codec() const;

    /* The members below are internal, not part of the public API, and
     * unsupported. Using them leads to undefined behavior. */
    virtual void item(const QPatternist::Item &item);
protected:
    QXmlSerializer(QAbstractXmlReceiverPrivate *d);

private:
    inline bool isBindingInScope(const QXmlName nb) const;

    /**
     * Where in the document the QXmlSerializer is currently working.
     */
    enum State
    {
        /**
         * Before the document element. This is the XML prolog where the
         * XML declaration, and possibly comments and processing
         * instructions are found.
         */
        BeforeDocumentElement,

        /**
         * This is inside the document element, at any level.
         */
        InsideDocumentElement
    };

    /**
     * If the current state is neither BeforeDocumentElement or
     * AfterDocumentElement.
     */
    inline bool atDocumentRoot() const;

    /**
     * Closes any open element start tag. Must be called before outputting
     * any element content.
     */
    inline void startContent();

    /**
     * Escapes content intended as text nodes for elements.
     */
    void writeEscaped(const QString &toEscape);

    /**
     * Identical to writeEscaped(), but also escapes quotes.
     */
    inline void writeEscapedAttribute(const QString &toEscape);

    /**
     * Writes out @p name.
     */
    inline void write(const QXmlName &name);

    inline void write(const char *const chars);
    /**
     * Encodes and writes out @p content.
     */
    inline void write(const QString &content);

    Q_DECLARE_PRIVATE(QXmlSerializer)
};

QT_END_NAMESPACE

#endif
