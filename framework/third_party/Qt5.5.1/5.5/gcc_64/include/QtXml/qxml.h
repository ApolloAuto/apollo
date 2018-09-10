/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the QtXml module of the Qt Toolkit.
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

#ifndef QXML_H
#define QXML_H

#include <QtXml/qtxmlglobal.h>
#include <QtCore/qtextstream.h>
#include <QtCore/qfile.h>
#include <QtCore/qstring.h>
#include <QtCore/qstringlist.h>
#include <QtCore/qlist.h>
#include <QtCore/qscopedpointer.h>

QT_BEGIN_NAMESPACE


class QXmlNamespaceSupport;
class QXmlAttributes;
class QXmlContentHandler;
class QXmlDefaultHandler;
class QXmlDTDHandler;
class QXmlEntityResolver;
class QXmlErrorHandler;
class QXmlLexicalHandler;
class QXmlDeclHandler;
class QXmlInputSource;
class QXmlLocator;
class QXmlNamespaceSupport;
class QXmlParseException;

class QXmlReader;
class QXmlSimpleReader;

class QXmlSimpleReaderPrivate;
class QXmlNamespaceSupportPrivate;
class QXmlAttributesPrivate;
class QXmlInputSourcePrivate;
class QXmlParseExceptionPrivate;
class QXmlLocatorPrivate;
class QXmlDefaultHandlerPrivate;


//
// SAX Namespace Support
//

class Q_XML_EXPORT QXmlNamespaceSupport
{
public:
    QXmlNamespaceSupport();
    ~QXmlNamespaceSupport();

    void setPrefix(const QString&, const QString&);

    QString prefix(const QString&) const;
    QString uri(const QString&) const;
    void splitName(const QString&, QString&, QString&) const;
    void processName(const QString&, bool, QString&, QString&) const;
    QStringList prefixes() const;
    QStringList prefixes(const QString&) const;

    void pushContext();
    void popContext();
    void reset();

private:
    QXmlNamespaceSupportPrivate *d;

    friend class QXmlSimpleReaderPrivate;
    Q_DISABLE_COPY(QXmlNamespaceSupport)
};


//
// SAX Attributes
//

class Q_XML_EXPORT QXmlAttributes
{
public:
    QXmlAttributes();
    virtual ~QXmlAttributes();

    int index(const QString& qName) const;
    int index(QLatin1String qName) const;
    int index(const QString& uri, const QString& localPart) const;
    int length() const;
    int count() const;
    QString localName(int index) const;
    QString qName(int index) const;
    QString uri(int index) const;
    QString type(int index) const;
    QString type(const QString& qName) const;
    QString type(const QString& uri, const QString& localName) const;
    QString value(int index) const;
    QString value(const QString& qName) const;
    QString value(QLatin1String qName) const;
    QString value(const QString& uri, const QString& localName) const;

    void clear();
    void append(const QString &qName, const QString &uri, const QString &localPart, const QString &value);

private:
    struct Attribute {
        QString qname, uri, localname, value;
    };
    typedef QList<Attribute> AttributeList;
    AttributeList attList;

    QXmlAttributesPrivate *d;
};

//
// SAX Input Source
//

class Q_XML_EXPORT QXmlInputSource
{
public:
    QXmlInputSource();
    explicit QXmlInputSource(QIODevice *dev);
    virtual ~QXmlInputSource();

    virtual void setData(const QString& dat);
    virtual void setData(const QByteArray& dat);
    virtual void fetchData();
    virtual QString data() const;
    virtual QChar next();
    virtual void reset();

    static const ushort EndOfData;
    static const ushort EndOfDocument;

protected:
    virtual QString fromRawData(const QByteArray &data, bool beginning = false);

private:
    void init();
    QXmlInputSourcePrivate *d;
};

//
// SAX Exception Classes
//

class Q_XML_EXPORT QXmlParseException
{
public:
    explicit QXmlParseException(const QString &name = QString(), int c = -1, int l = -1,
                                const QString &p = QString(), const QString &s = QString());
    QXmlParseException(const QXmlParseException &other);
    ~QXmlParseException();

    int columnNumber() const;
    int lineNumber() const;
    QString publicId() const;
    QString systemId() const;
    QString message() const;

private:
    QScopedPointer<QXmlParseExceptionPrivate> d;
};


//
// XML Reader
//

class Q_XML_EXPORT QXmlReader
{
public:
    virtual ~QXmlReader() {}
    virtual bool feature(const QString& name, bool *ok = 0) const = 0;
    virtual void setFeature(const QString& name, bool value) = 0;
    virtual bool hasFeature(const QString& name) const = 0;
    virtual void* property(const QString& name, bool *ok = 0) const = 0;
    virtual void setProperty(const QString& name, void* value) = 0;
    virtual bool hasProperty(const QString& name) const = 0;
    virtual void setEntityResolver(QXmlEntityResolver* handler) = 0;
    virtual QXmlEntityResolver* entityResolver() const = 0;
    virtual void setDTDHandler(QXmlDTDHandler* handler) = 0;
    virtual QXmlDTDHandler* DTDHandler() const = 0;
    virtual void setContentHandler(QXmlContentHandler* handler) = 0;
    virtual QXmlContentHandler* contentHandler() const = 0;
    virtual void setErrorHandler(QXmlErrorHandler* handler) = 0;
    virtual QXmlErrorHandler* errorHandler() const = 0;
    virtual void setLexicalHandler(QXmlLexicalHandler* handler) = 0;
    virtual QXmlLexicalHandler* lexicalHandler() const = 0;
    virtual void setDeclHandler(QXmlDeclHandler* handler) = 0;
    virtual QXmlDeclHandler* declHandler() const = 0;
    virtual bool parse(const QXmlInputSource& input) = 0;
    virtual bool parse(const QXmlInputSource* input) = 0;
};

class Q_XML_EXPORT QXmlSimpleReader : public QXmlReader
{
public:
    QXmlSimpleReader();
    virtual ~QXmlSimpleReader();

    bool feature(const QString& name, bool *ok = 0) const Q_DECL_OVERRIDE;
    void setFeature(const QString& name, bool value) Q_DECL_OVERRIDE;
    bool hasFeature(const QString& name) const Q_DECL_OVERRIDE;

    void* property(const QString& name, bool *ok = 0) const Q_DECL_OVERRIDE;
    void setProperty(const QString& name, void* value) Q_DECL_OVERRIDE;
    bool hasProperty(const QString& name) const Q_DECL_OVERRIDE;

    void setEntityResolver(QXmlEntityResolver* handler) Q_DECL_OVERRIDE;
    QXmlEntityResolver* entityResolver() const Q_DECL_OVERRIDE;
    void setDTDHandler(QXmlDTDHandler* handler) Q_DECL_OVERRIDE;
    QXmlDTDHandler* DTDHandler() const Q_DECL_OVERRIDE;
    void setContentHandler(QXmlContentHandler* handler) Q_DECL_OVERRIDE;
    QXmlContentHandler* contentHandler() const Q_DECL_OVERRIDE;
    void setErrorHandler(QXmlErrorHandler* handler) Q_DECL_OVERRIDE;
    QXmlErrorHandler* errorHandler() const Q_DECL_OVERRIDE;
    void setLexicalHandler(QXmlLexicalHandler* handler) Q_DECL_OVERRIDE;
    QXmlLexicalHandler* lexicalHandler() const Q_DECL_OVERRIDE;
    void setDeclHandler(QXmlDeclHandler* handler) Q_DECL_OVERRIDE;
    QXmlDeclHandler* declHandler() const Q_DECL_OVERRIDE;

    bool parse(const QXmlInputSource& input) Q_DECL_OVERRIDE;
    bool parse(const QXmlInputSource* input) Q_DECL_OVERRIDE;
    virtual bool parse(const QXmlInputSource* input, bool incremental);
    virtual bool parseContinue();

private:
    Q_DISABLE_COPY(QXmlSimpleReader)
    Q_DECLARE_PRIVATE(QXmlSimpleReader)
    QScopedPointer<QXmlSimpleReaderPrivate> d_ptr;

    friend class QXmlSimpleReaderLocator;
    friend class QDomHandler;
};

//
// SAX Locator
//

class Q_XML_EXPORT QXmlLocator
{
public:
    QXmlLocator();
    virtual ~QXmlLocator();

    virtual int columnNumber() const = 0;
    virtual int lineNumber() const = 0;
//    QString getPublicId() const
//    QString getSystemId() const
};

//
// SAX handler classes
//

class Q_XML_EXPORT QXmlContentHandler
{
public:
    virtual ~QXmlContentHandler() {}
    virtual void setDocumentLocator(QXmlLocator* locator) = 0;
    virtual bool startDocument() = 0;
    virtual bool endDocument() = 0;
    virtual bool startPrefixMapping(const QString& prefix, const QString& uri) = 0;
    virtual bool endPrefixMapping(const QString& prefix) = 0;
    virtual bool startElement(const QString& namespaceURI, const QString& localName, const QString& qName, const QXmlAttributes& atts) = 0;
    virtual bool endElement(const QString& namespaceURI, const QString& localName, const QString& qName) = 0;
    virtual bool characters(const QString& ch) = 0;
    virtual bool ignorableWhitespace(const QString& ch) = 0;
    virtual bool processingInstruction(const QString& target, const QString& data) = 0;
    virtual bool skippedEntity(const QString& name) = 0;
    virtual QString errorString() const = 0;
};

class Q_XML_EXPORT QXmlErrorHandler
{
public:
    virtual ~QXmlErrorHandler() {}
    virtual bool warning(const QXmlParseException& exception) = 0;
    virtual bool error(const QXmlParseException& exception) = 0;
    virtual bool fatalError(const QXmlParseException& exception) = 0;
    virtual QString errorString() const = 0;
};

class Q_XML_EXPORT QXmlDTDHandler
{
public:
    virtual ~QXmlDTDHandler() {}
    virtual bool notationDecl(const QString& name, const QString& publicId, const QString& systemId) = 0;
    virtual bool unparsedEntityDecl(const QString& name, const QString& publicId, const QString& systemId, const QString& notationName) = 0;
    virtual QString errorString() const = 0;
};

class Q_XML_EXPORT QXmlEntityResolver
{
public:
    virtual ~QXmlEntityResolver() {}
    virtual bool resolveEntity(const QString& publicId, const QString& systemId, QXmlInputSource*& ret) = 0;
    virtual QString errorString() const = 0;
};

class Q_XML_EXPORT QXmlLexicalHandler
{
public:
    virtual ~QXmlLexicalHandler() {}
    virtual bool startDTD(const QString& name, const QString& publicId, const QString& systemId) = 0;
    virtual bool endDTD() = 0;
    virtual bool startEntity(const QString& name) = 0;
    virtual bool endEntity(const QString& name) = 0;
    virtual bool startCDATA() = 0;
    virtual bool endCDATA() = 0;
    virtual bool comment(const QString& ch) = 0;
    virtual QString errorString() const = 0;
};

class Q_XML_EXPORT QXmlDeclHandler
{
public:
    virtual ~QXmlDeclHandler() {}
    virtual bool attributeDecl(const QString& eName, const QString& aName, const QString& type, const QString& valueDefault, const QString& value) = 0;
    virtual bool internalEntityDecl(const QString& name, const QString& value) = 0;
    virtual bool externalEntityDecl(const QString& name, const QString& publicId, const QString& systemId) = 0;
    virtual QString errorString() const = 0;
    // ### Conform to SAX by adding elementDecl
};


class Q_XML_EXPORT QXmlDefaultHandler : public QXmlContentHandler, public QXmlErrorHandler, public QXmlDTDHandler, public QXmlEntityResolver, public QXmlLexicalHandler, public QXmlDeclHandler
{
public:
    QXmlDefaultHandler();
    virtual ~QXmlDefaultHandler();

    void setDocumentLocator(QXmlLocator* locator) Q_DECL_OVERRIDE;
    bool startDocument() Q_DECL_OVERRIDE;
    bool endDocument() Q_DECL_OVERRIDE;
    bool startPrefixMapping(const QString& prefix, const QString& uri) Q_DECL_OVERRIDE;
    bool endPrefixMapping(const QString& prefix) Q_DECL_OVERRIDE;
    bool startElement(const QString& namespaceURI, const QString& localName, const QString& qName, const QXmlAttributes& atts) Q_DECL_OVERRIDE;
    bool endElement(const QString& namespaceURI, const QString& localName, const QString& qName) Q_DECL_OVERRIDE;
    bool characters(const QString& ch) Q_DECL_OVERRIDE;
    bool ignorableWhitespace(const QString& ch) Q_DECL_OVERRIDE;
    bool processingInstruction(const QString& target, const QString& data) Q_DECL_OVERRIDE;
    bool skippedEntity(const QString& name) Q_DECL_OVERRIDE;

    bool warning(const QXmlParseException& exception) Q_DECL_OVERRIDE;
    bool error(const QXmlParseException& exception) Q_DECL_OVERRIDE;
    bool fatalError(const QXmlParseException& exception) Q_DECL_OVERRIDE;

    bool notationDecl(const QString& name, const QString& publicId, const QString& systemId) Q_DECL_OVERRIDE;
    bool unparsedEntityDecl(const QString& name, const QString& publicId, const QString& systemId, const QString& notationName) Q_DECL_OVERRIDE;

    bool resolveEntity(const QString& publicId, const QString& systemId, QXmlInputSource*& ret) Q_DECL_OVERRIDE;

    bool startDTD(const QString& name, const QString& publicId, const QString& systemId) Q_DECL_OVERRIDE;
    bool endDTD() Q_DECL_OVERRIDE;
    bool startEntity(const QString& name) Q_DECL_OVERRIDE;
    bool endEntity(const QString& name) Q_DECL_OVERRIDE;
    bool startCDATA() Q_DECL_OVERRIDE;
    bool endCDATA() Q_DECL_OVERRIDE;
    bool comment(const QString& ch) Q_DECL_OVERRIDE;

    bool attributeDecl(const QString& eName, const QString& aName, const QString& type, const QString& valueDefault, const QString& value) Q_DECL_OVERRIDE;
    bool internalEntityDecl(const QString& name, const QString& value) Q_DECL_OVERRIDE;
    bool externalEntityDecl(const QString& name, const QString& publicId, const QString& systemId) Q_DECL_OVERRIDE;

    QString errorString() const Q_DECL_OVERRIDE;

private:
    QXmlDefaultHandlerPrivate *d;
    Q_DISABLE_COPY(QXmlDefaultHandler)
};

// inlines

inline int QXmlAttributes::count() const
{ return length(); }

QT_END_NAMESPACE

#endif // QXML_H
