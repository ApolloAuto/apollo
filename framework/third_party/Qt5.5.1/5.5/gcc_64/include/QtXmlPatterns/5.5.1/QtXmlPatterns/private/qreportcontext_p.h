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

#ifndef Patternist_ReportContext_H
#define Patternist_ReportContext_H

#include <QSharedData>
#include <QAbstractUriResolver>
#include <QSourceLocation>

#include <private/qnamepool_p.h>
#include <QXmlName>

QT_BEGIN_NAMESPACE

class QAbstractMessageHandler;
class QSourceLocation;
class QString;

namespace QPatternist
{
    class SourceLocationReflection;

    /**
     * @short A callback for reporting errors.
     *
     * ReportContext receives messages of various severity and type via its
     * functions warning() and error(). In turn, ReportContext create Message instances
     * and submit them to the QAbstractMessageHandler instance returned by messageHandler().
     *
     * The Message attributes are set as follows:
     *
     * - Message::description() - A translated, human-readable description
     * - Message::type() - Message::Error if a static, dynamic or type error was encountered
     * that halted compilation or evaluation, or Message::Warning in case of a warning
     * - Message::identifier() - This is a URI consisting of the error namespace with the
     * error code as fragment. For example, a Message representing a syntax error
     * would return the type "http://www.w3.org/2005/xqt-errors#XPST0003". The convenience
     * function codeFromURI() can be used to extract the error code. The error namespace
     * is typically the namespace for XPath and XQuery errors(as in the previous example), but
     * can also be user defined.
     *
     * @see <a href="http://www.w3.org/TR/xpath20/#id-identifying-errors">XML Path Language
     * (XPath) 2.0, 2.3.2 Identifying and Reporting Errors</a>
     * @see <a href="http://www.w3.org/TR/xpath-functions/#func-error">XQuery 1.0 and
     * XPath 2.0 Functions and Operators, 3 The Error Function</a>
     * @author Frans Englich <frans.englich@nokia.com>
     * @warning This file is auto-generated from extractErrorCodes.xsl. Any
     * modifications done to this file are lost.
     */
    class Q_AUTOTEST_EXPORT ReportContext : public QSharedData
    {
    public:
        typedef QHash<const SourceLocationReflection *, QSourceLocation> LocationHash;

        /**
         * A smart pointer wrapping ReportContext instances.
         */
        typedef QExplicitlySharedDataPointer<ReportContext> Ptr;

        /**
         * @short Default constructors.
         *
         * For some reason GCC fails to synthesize it, so we provide an empty
         * one here.
         */
        inline ReportContext() {}

        virtual ~ReportContext();

        /**
         * Error codes that corresponds to the error codes defined in the
         * relevant specifications. They are used throughout the API for
         * identifying error conditions.
         *
         * While strings could have been used for identifying errors, enums
         * reduces bugs by providing type safety.
         *
         * @see <a href="http://www.w3.org/TR/xpath20/#errors">XML
         * Path Language (XPath) 2.0, 2.3 Error Handling</a>
         * @see <a href="http://www.w3.org/TR/xpath-functions/#d1e10985">XQuery 1.0
         * and XPath 2.0 Functions and Operators, C Error Summary</a>
         * @see <a href="http://www.w3.org/TR/xslt20/#error-summary">XSL Transformations
         * (XSLT) Version 2.0, E Summary of Error Conditions (Non-Normative)</a>
         * @note The enumerator values' Doxygen documentation is copied from the
         * W3C documents
         * <a href="http://www.w3.org/TR/xpath-functions">XQuery 1.0 and XPath
         * 2.0 Functions and Operators</a>,
         * <a href="http://www.w3.org/TR/xpath20">XML Path Language (XPath) 2.0</a>, and
         * <a href="http://www.w3.org/TR/xslt20/">XSL Transformations (XSLT)
         * Version 2.0</a>, respectively. The doxygen documentation is therefore covered
         * by the following legal notice:
         * "Copyright @ 2005 W3C&reg; (MIT, ERCIM, Keio), All Rights Reserved. W3C
         * <a href="http://www.w3.org/Consortium/Legal/ipr-notice#Legal_Disclaimer">liability</a>,
         * <a href="http://www.w3.org/Consortium/Legal/ipr-notice#W3C_Trademarks">trademark</a> and
         * <a href="http://www.w3.org/Consortium/Legal/copyright-documents">document
         * use</a> rules apply."
         * @warning This enumerator is auto-generated from the relevant specifications
         * by the XSL-T stylesheet extractErrorCodes.xsl. Hence, any modifications
         * done to this file, in contrary to the stylesheet, are therefore lost.
         */
        enum ErrorCode
        {
            /**
             * XML Schema error code.
             */
            XSDError,

            /**
             * It is a static error if analysis of an expression relies on some
             * component of the static context that has not been assigned a
             * value.
             */
            XPST0001,

            /**
             * It is a dynamic error if evaluation of an expression relies on
             * some part of the dynamic context that has not been assigned a
             * value.
             */
            XPDY0002,

            /**
             * It is a static error if an expression is not a valid instance
             * of the grammar defined in A.1 EBNF.
             */
            XPST0003,

            /**
             * It is a type error if, during the static analysis phase, an expression
             * is found to have a static type that is not appropriate for the
             * context in which the expression occurs, or during the dynamic
             * evaluation phase, the dynamic type of a value does not match
             * a required type as specified by the matching rules in 2.5.4 SequenceType
             * Matching.
             */
            XPTY0004,

            /**
             * During the analysis phase, it is a static error if the static
             * type assigned to an expression other than the expression () or
             * data(()) is empty-sequence().
             */
            XPST0005,

            /**
             * (Not currently used.)
             */
            XPTY0006,

            /**
             * (Not currently used.)
             */
            XPTY0007,

            /**
             * It is a static error if an expression refers to an element name,
             * attribute name, schema type name, namespace prefix, or variable
             * name that is not defined in the static context, except for an
             * ElementName in an ElementTest or an AttributeName in an AttributeTest.
             */
            XPST0008,

            /**
             * An implementation that does not support the Schema Import Feature
             * must raise a static error if a Prolog contains a schema import.
             */
            XQST0009,

            /**
             * An implementation must raise a static error if it encounters
             * a reference to an axis that it does not support.
             */
            XPST0010,

            /**
             * It is a static error if the set of definitions contained in all
             * schemas imported by a Prolog do not satisfy the conditions for
             * schema validity specified in Sections 3 and 5 of [XML Schema]
             * Part 1--i.e., each definition must be valid, complete, and unique.
             */
            XQST0012,

            /**
             * It is a static error if an implementation recognizes a pragma
             * but determines that its content is invalid.
             */
            XQST0013,

            /**
             * (Not currently used.)
             */
            XQST0014,

            /**
             * (Not currently used.)
             */
            XQST0015,

            /**
             * An implementation that does not support the Module Feature raises
             * a static error if it encounters a module declaration or a module
             * import.
             */
            XQST0016,

            /**
             * It is a static error if the expanded QName and number of arguments
             * in a function call do not match the name and arity of a function
             * signature in the static context.
             */
            XPST0017,

            /**
             * It is a type error if the result of the last step in a path expression
             * contains both nodes and atomic values.
             */
            XPTY0018,

            /**
             * It is a type error if the result of a step (other than the last
             * step) in a path expression contains an atomic value.
             */
            XPTY0019,

            /**
             * It is a type error if, in an axis step, the context item is not
             * a node.
             */
            XPTY0020,

            /**
             * (Not currently used.)
             */
            XPDY0021,

            /**
             * It is a static error if the value of a namespace declaration
             * attribute is not a URILiteral.
             */
            XQST0022,

            /**
             * (Not currently used.)
             */
            XQTY0023,

            /**
             * It is a type error if the content sequence in an element constructor
             * contains an attribute node following a node that is not an attribute
             * node.
             */
            XQTY0024,

            /**
             * It is a dynamic error if any attribute of a constructed element
             * does not have a name that is distinct from the names of all other
             * attributes of the constructed element.
             */
            XQDY0025,

            /**
             * It is a dynamic error if the result of the content expression
             * of a computed processing instruction constructor contains the
             * string "?&gt;".
             */
            XQDY0026,

            /**
             * In a validate expression, it is a dynamic error if the root element
             * information item in the PSVI resulting from validation does not
             * have the expected validity property: valid if validation mode
             * is strict, or either valid or notKnown if validation mode is
             * lax.
             */
            XQDY0027,

            /**
             * (Not currently used.)
             */
            XQTY0028,

            /**
             * (Not currently used.)
             */
            XQDY0029,

            /**
             * It is a type error if the argument of a validate expression does
             * not evaluate to exactly one document or element node.
             */
            XQTY0030,

            /**
             * It is a static error if the version number specified in a version
             * declaration is not supported by the implementation.
             */
            XQST0031,

            /**
             * A static error is raised if a Prolog contains more than one base
             * URI declaration.
             */
            XQST0032,

            /**
             * It is a static error if a module contains multiple bindings for
             * the same namespace prefix.
             */
            XQST0033,

            /**
             * It is a static error if multiple functions declared or imported
             * by a module have the number of arguments and their expanded QNames
             * are equal (as defined by the eq operator).
             */
            XQST0034,

            /**
             * It is a static error to import two schema components that both
             * define the same name in the same symbol space and in the same
             * scope.
             */
            XQST0035,

            /**
             * It is a static error to import a module if the importing module's
             * in-scope schema types do not include definitions for the schema
             * type names that appear in the declarations of variables and functions
             * (whether in an argument type or return type) that are present
             * in the imported module and are referenced in the importing module.
             */
            XQST0036,

            /**
             * (Not currently used.)
             */
            XQST0037,

            /**
             * It is a static error if a Prolog contains more than one default
             * collation declaration, or the value specified by a default collation
             * declaration is not present in statically known collations.
             */
            XQST0038,

            /**
             * It is a static error for a function declaration to have more
             * than one parameter with the same name.
             */
            XQST0039,

            /**
             * It is a static error if the attributes specified by a direct
             * element constructor do not have distinct expanded QNames.
             */
            XQST0040,

            /**
             * It is a dynamic error if the value of the name expression in
             * a computed processing instruction constructor cannot be cast
             * to the type xs:NCName.
             */
            XQDY0041,

            /**
             * (Not currently used.)
             */
            XQST0042,

            /**
             * (Not currently used.)
             */
            XQST0043,

            /**
             * It is a dynamic error if the node-name property of the node constructed
             * by a computed attribute constructor is in the namespace http://www.w3.org/2000/xmlns/
             * (corresponding to namespace prefix xmlns), or is in no namespace
             * and has local name xmlns.
             */
            XQDY0044,

            /**
             * It is a static error if the function name in a function declaration
             * is in one of the following namespaces: http://www.w3.org/XML/1998/namespace,
             * http://www.w3.org/2001/XMLSchema, http://www.w3.org/2001/XMLSchema-instance,
             * http://www.w3.org/2005/xpath-functions.
             */
            XQST0045,

            /**
             * An implementation MAY raise a static error if the value of a
             * URILiteral is of nonzero length and is not in the lexical space
             * of xs:anyURI.
             */
            XQST0046,

            /**
             * It is a static error if multiple module imports in the same Prolog
             * specify the same target namespace.
             */
            XQST0047,

            /**
             * It is a static error if a function or variable declared in a
             * library module is not in the target namespace of the library
             * module.
             */
            XQST0048,

            /**
             * It is a static error if two or more variables declared or imported
             * by a module have equal expanded QNames (as defined by the eq
             * operator.)
             */
            XQST0049,

            /**
             * It is a dynamic error if the dynamic type of the operand of a
             * treat expression does not match the sequence type specified by
             * the treat expression. This error might also be raised by a path
             * expression beginning with "/" or "//" if the context node is
             * not in a tree that is rooted at a document node. This is because
             * a leading "/" or "//" in a path expression is an abbreviation
             * for an initial step that includes the clause treat as document-node().
             */
            XPDY0050,

            /**
             * It is a static error if a QName that is used as an AtomicType
             * in a SequenceType is not defined in the in-scope schema types
             * as an atomic type.
             */
            XPST0051,

            /**
             * (Not currently used.)
             */
            XQDY0052,

            /**
             * (Not currently used.)
             */
            XQST0053,

            /**
             * It is a static error if a variable depends on itself.
             */
            XQST0054,

            /**
             * It is a static error if a Prolog contains more than one copy-namespaces
             * declaration.
             */
            XQST0055,

            /**
             * (Not currently used.)
             */
            XQST0056,

            /**
             * It is a static error if a schema import binds a namespace prefix
             * but does not specify a target namespace other than a zero-length
             * string.
             */
            XQST0057,

            /**
             * It is a static error if multiple schema imports specify the same
             * target namespace.
             */
            XQST0058,

            /**
             * It is a static error if an implementation is unable to process
             * a schema or module import by finding a schema or module with
             * the specified target namespace.
             */
            XQST0059,

            /**
             * It is a static error if the name of a function in a function
             * declaration is not in a namespace (expanded QName has a null
             * namespace URI).
             */
            XQST0060,

            /**
             * It is a dynamic error if the operand of a validate expression
             * is a document node whose children do not consist of exactly one
             * element node and zero or more comment and processing instruction
             * nodes, in any order.
             */
            XQDY0061,

            /**
             * (Not currently used.)
             */
            XQDY0062,

            /**
             * (Not currently used.)
             */
            XQST0063,

            /**
             * It is a dynamic error if the value of the name expression in
             * a computed processing instruction constructor is equal to "XML"
             * (in any combination of upper and lower case).
             */
            XQDY0064,

            /**
             * A static error is raised if a Prolog contains more than one ordering
             * mode declaration.
             */
            XQST0065,

            /**
             * A static error is raised if a Prolog contains more than one default
             * element/type namespace declaration, or more than one default
             * function namespace declaration.
             */
            XQST0066,

            /**
             * A static error is raised if a Prolog contains more than one construction
             * declaration.
             */
            XQST0067,

            /**
             * A static error is raised if a Prolog contains more than one boundary-space
             * declaration.
             */
            XQST0068,

            /**
             * A static error is raised if a Prolog contains more than one empty
             * order declaration.
             */
            XQST0069,

            /**
             * A static error is raised if a namespace URI is bound to the predefined
             * prefix xmlns, or if a namespace URI other than http://www.w3.org/XML/1998/namespace
             * is bound to the prefix xml, or if the prefix xml is bound to
             * a namespace URI other than http://www.w3.org/XML/1998/namespace.
             */
            XQST0070,

            /**
             * A static error is raised if the namespace declaration attributes
             * of a direct element constructor do not have distinct names.
             */
            XQST0071,

            /**
             * It is a dynamic error if the result of the content expression
             * of a computed comment constructor contains two adjacent hyphens
             * or ends with a hyphen.
             */
            XQDY0072,

            /**
             * It is a static error if the graph of module imports contains
             * a cycle (that is, if there exists a sequence of modules M1 ...
             * Mn such that each Mi imports Mi+1 and Mn imports M1), unless
             * all the modules in the cycle share a common namespace.
             */
            XQST0073,

            /**
             * It is a dynamic error if the value of the name expression in
             * a computed element or attribute constructor cannot be converted
             * to an expanded QName (for example, because it contains a namespace
             * prefix not found in statically known namespaces.)
             */
            XQDY0074,

            /**
             * An implementation that does not support the Validation Feature
             * must raise a static error if it encounters a validate expression.
             */
            XQST0075,

            /**
             * It is a static error if a collation subclause in an order by
             * clause of a FLWOR expression does not identify a collation that
             * is present in statically known collations.
             */
            XQST0076,

            /**
             * (Not currently used.)
             */
            XQST0077,

            /**
             * (Not currently used.)
             */
            XQST0078,

            /**
             * It is a static error if an extension expression contains neither
             * a pragma that is recognized by the implementation nor an expression
             * enclosed in curly braces.
             */
            XQST0079,

            /**
             * It is a static error if the target type of a cast or castable
             * expression is xs:NOTATION or xs:anyAtomicType.
             */
            XPST0080,

            /**
             * It is a static error if a QName used in a query contains a namespace
             * prefix that cannot be expanded into a namespace URI by using
             * the statically known namespaces.
             */
            XPST0081,

            /**
             * (Not currently used.)
             */
            XQST0082,

            /**
             * (Not currently used.)
             */
            XPST0083,

            /**
             * It is a dynamic error if the element validated by a validate
             * statement does not have a top-level element declaration in the
             * in-scope element declarations, if validation mode is strict.
             */
            XQDY0084,

            /**
             * It is a static error if the namespace URI in a namespace declaration
             * attribute is a zero-length string, and the implementation does
             * not support [XML Names 1.1].
             */
            XQST0085,

            /**
             * It is a type error if the typed value of a copied element or
             * attribute node is namespace-sensitive when construction mode
             * is preserve and copy-namespaces mode is no-preserve.
             */
            XQTY0086,

            /**
             * It is a static error if the encoding specified in a Version Declaration
             * does not conform to the definition of EncName specified in [XML
             * 1.0].
             */
            XQST0087,

            /**
             * It is a static error if the literal that specifies the target
             * namespace in a module import or a module declaration is of zero
             * length.
             */
            XQST0088,

            /**
             * It is a static error if a variable bound in a for clause of a
             * FLWOR expression, and its associated positional variable, do
             * not have distinct names (expanded QNames).
             */
            XQST0089,

            /**
             * It is a static error if a character reference does not identify
             * a valid character in the version of XML that is in use.
             */
            XQST0090,

            /**
             * An implementation MAY raise a dynamic error if an xml:id error,
             * as defined in [XML ID], is encountered during construction of
             * an attribute named xml:id.
             */
            XQDY0091,

            /**
             * An implementation MAY raise a dynamic error if a constructed
             * attribute named xml:space has a value other than preserve or
             * default.
             */
            XQDY0092,

            /**
             * It is a static error to import a module M1 if there exists a
             * sequence of modules M1 ... Mi ... M1 such that each module directly
             * depends on the next module in the sequence (informally, if M1
             * depends on itself through some chain of module dependencies.)
             */
            XQST0093,

            /**
             * Unidentified error.
             */
            FOER0000,

            /**
             * Division by zero.
             */
            FOAR0001,

            /**
             * Numeric operation overflow/underflow.
             */
            FOAR0002,

            /**
             * Input value too large for decimal.
             */
            FOCA0001,

            /**
             * Invalid lexical value.
             */
            FOCA0002,

            /**
             * Input value too large for integer.
             */
            FOCA0003,

            /**
             * NaN supplied as float/double value.
             */
            FOCA0005,

            /**
             * String to be cast to decimal has too many digits of precision.
             */
            FOCA0006,

            /**
             * Code point not valid.
             */
            FOCH0001,

            /**
             * Unsupported collation.
             */
            FOCH0002,

            /**
             * Unsupported normalization form.
             */
            FOCH0003,

            /**
             * Collation does not support collation units.
             */
            FOCH0004,

            /**
             * No context document.
             */
            FODC0001,

            /**
             * Error retrieving resource.
             */
            FODC0002,

            /**
             * Function stability not defined.
             */
            FODC0003,

            /**
             * Invalid argument to fn:collection.
             */
            FODC0004,

            /**
             * Invalid argument to fn:doc or fn:doc-available.
             */
            FODC0005,

            /**
             * Overflow/underflow in date/time operation.
             */
            FODT0001,

            /**
             * Overflow/underflow in duration operation.
             */
            FODT0002,

            /**
             * Invalid timezone value.
             */
            FODT0003,

            /**
             * No namespace found for prefix.
             */
            FONS0004,

            /**
             * Base-uri not defined in the static context.
             */
            FONS0005,

            /**
             * Invalid value for cast/constructor.
             */
            FORG0001,

            /**
             * Invalid argument to fn:resolve-uri().
             */
            FORG0002,

            /**
             * fn:zero-or-one called with a sequence containing more than one
             * item.
             */
            FORG0003,

            /**
             * fn:one-or-more called with a sequence containing no items.
             */
            FORG0004,

            /**
             * fn:exactly-one called with a sequence containing zero or more
             * than one item.
             */
            FORG0005,

            /**
             * Invalid argument type.
             */
            FORG0006,

            /**
             * Both arguments to fn:dateTime have a specified timezone.
             */
            FORG0008,

            /**
             * Error in resolving a relative URI against a base URI in fn:resolve-uri.
             */
            FORG0009,

            /**
             * Invalid regular expression. flags
             */
            FORX0001,

            /**
             * Invalid regular expression.
             */
            FORX0002,

            /**
             * Regular expression matches zero-length string.
             */
            FORX0003,

            /**
             * Invalid replacement string.
             */
            FORX0004,

            /**
             * Argument node does not have a typed value.
             */
            FOTY0012,

            /**
             * It is an error if an item in S6 in sequence normalization is
             * an attribute node or a namespace node.
             */
            SENR0001,

            /**
             * It is an error if the serializer is unable to satisfy the rules
             * for either a well-formed XML document entity or a well-formed
             * XML external general parsed entity, or both, except for content
             * modified by the character expansion phase of serialization.
             */
            SERE0003,

            /**
             * It is an error to specify the doctype-system parameter, or to
             * specify the standalone parameter with a value other than omit,
             * if the instance of the data model contains text nodes or multiple
             * element nodes as children of the root node.
             */
            SEPM0004,

            /**
             * It is an error if the serialized result would contain an NCName
             * Names that contains a character that is not permitted by the
             * version of Namespaces in XML specified by the version parameter.
             */
            SERE0005,

            /**
             * It is an error if the serialized result would contain a character
             * that is not permitted by the version of XML specified by the
             * version parameter.
             */
            SERE0006,

            /**
             * It is an error if an output encoding other than UTF-8 or UTF-16
             * is requested and the serializer does not support that encoding.
             */
            SESU0007,

            /**
             * It is an error if a character that cannot be represented in the
             * encoding that the serializer is using for output appears in a
             * context where character references are not allowed (for example
             * if the character occurs in the name of an element).
             */
            SERE0008,

            /**
             * It is an error if the omit-xml-declaration parameter has the
             * value yes, and the standalone attribute has a value other than
             * omit; or the version parameter has a value other than 1.0 and
             * the doctype-system parameter is specified.
             */
            SEPM0009,

            /**
             * It is an error if the output method is xml, the value of the
             * undeclare-prefixes parameter is yes, and the value of the version
             * parameter is 1.0.
             */
            SEPM0010,

            /**
             * It is an error if the value of the normalization-form parameter
             * specifies a normalization form that is not supported by the serializer.
             */
            SESU0011,

            /**
             * It is an error if the value of the normalization-form parameter
             * is fully-normalized and any relevant construct of the result
             * begins with a combining character.
             */
            SERE0012,

            /**
             * It is an error if the serializer does not support the version
             * of XML or HTML specified by the version parameter.
             */
            SESU0013,

            /**
             * It is an error to use the HTML output method when characters
             * which are legal in XML but not in HTML, specifically the control
             * characters \#x7F-#x9F, appear in the instance of the data model.
             */
            SERE0014,

            /**
             * It is an error to use the HTML output method when &gt; appears within
             * a processing instruction in the data model instance being serialized.
             */
            SERE0015,

            /**
             * It is a an error if a parameter value is invalid for the defined
             * domain.
             */
            SEPM0016,

            /**
             * A static error is signaled if an XSLT-defined element is used
             * in a context where it is not permitted, if a required attribute
             * is omitted, or if the content of the element does not correspond
             * to the content that is allowed for the element.
             */
            XTSE0010,

            /**
             * It is a static error if an attribute (other than an attribute
             * written using curly brackets in a position where an attribute
             * value template is permitted) contains a value that is not one
             * of the permitted values for that attribute.
             */
            XTSE0020,

            /**
             * It is a static error to use a reserved namespace in the name
             * of a named template, a mode, an attribute set, a key, a decimal-format,
             * a variable or parameter, a stylesheet function, a named output
             * definition, or a character map.
             */
            XTSE0080,

            /**
             * It is a static error for an element from the XSLT namespace to
             * have an attribute whose namespace is either null (that is, an
             * attribute with an unprefixed name) or the XSLT namespace, other
             * than attributes defined for the element in this document.
             */
            XTSE0090,

            /**
             * The value of the version attribute must be a number: specifically,
             * it must be a a valid instance of the type xs:decimal as defined
             * in [XML Schema Part 2].
             */
            XTSE0110,

            /**
             * An xsl:stylesheet element must not have any text node children.
             */
            XTSE0120,

            /**
             * It is a static error if the value of an [xsl:]default-collation
             * attribute, after resolving against the base URI, contains no
             * URI that the implementation recognizes as a collation URI.
             */
            XTSE0125,

            /**
             * It is a static error if the xsl:stylesheet element has a child
             * element whose name has a null namespace URI.
             */
            XTSE0130,

            /**
             * A literal result element that is used as the outermost element
             * of a simplified stylesheet module must have an xsl:version attribute.
             */
            XTSE0150,

            /**
             * It is a static error if the processor is not able to retrieve
             * the resource identified by the URI reference [ in the href attribute
             * of xsl:include or xsl:import] , or if the resource that is retrieved
             * does not contain a stylesheet module conforming to this specification.
             */
            XTSE0165,

            /**
             * An xsl:include element must be a top-level element.
             */
            XTSE0170,

            /**
             * It is a static error if a stylesheet module directly or indirectly
             * includes itself.
             */
            XTSE0180,

            /**
             * An xsl:import element must be a top-level element.
             */
            XTSE0190,

            /**
             * The xsl:import element children must precede all other element
             * children of an xsl:stylesheet element, including any xsl:include
             * element children and any user-defined data elements.
             */
            XTSE0200,

            /**
             * It is a static error if a stylesheet module directly or indirectly
             * imports itself.
             */
            XTSE0210,

            /**
             * It is a static error if an xsl:import-schema element that contains
             * an xs:schema element has a schema-location attribute, or if it
             * has a namespace attribute that conflicts with the target namespace
             * of the contained schema.
             */
            XTSE0215,

            /**
             * It is a static error if the synthetic schema document does not
             * satisfy the constraints described in [XML Schema Part 1] (section
             * 5.1, Errors in Schema Construction and Structure). This includes,
             * without loss of generality, conflicts such as multiple definitions
             * of the same name.
             */
            XTSE0220,

            /**
             * Within an XSLT element that is required to be empty, any content
             * other than comments or processing instructions, including any
             * whitespace text node preserved using the xml:space="preserve"
             * attribute, is a static error.
             */
            XTSE0260,

            /**
             * It is a static error if there is a stylesheet module in the stylesheet
             * that specifies input-type-annotations="strip" and another stylesheet
             * module that specifies input-type-annotations="preserve".
             */
            XTSE0265,

            /**
             * In the case of a prefixed QName used as the value of an attribute
             * in the stylesheet, or appearing within an XPath expression in
             * the stylesheet, it is a static error if the defining element
             * has no namespace node whose name matches the prefix of the QName.
             */
            XTSE0280,

            /**
             * Where an attribute is defined to contain a pattern, it is a static
             * error if the pattern does not match the production Pattern.
             */
            XTSE0340,

            /**
             * It is a static error if an unescaped left curly bracket appears
             * in a fixed part of an attribute value template without a matching
             * right curly bracket.
             */
            XTSE0350,

            /**
             * It is a static error if an unescaped right curly bracket occurs
             * in a fixed part of an attribute value template.
             */
            XTSE0370,

            /**
             * An xsl:template element must have either a match attribute or
             * a name attribute, or both. An xsl:template element that has no
             * match attribute must have no mode attribute and no priority attribute.
             */
            XTSE0500,

            /**
             * The value of this attribute [the priority attribute of the xsl:template
             * element] must conform to the rules for the xs:decimal type defined
             * in [XML Schema Part 2]. Negative values are permitted..
             */
            XTSE0530,

            /**
             * It is a static error if the list [of modes in the mode attribute
             * of xsl:template] is empty, if the same token is included more
             * than once in the list, if the list contains an invalid token,
             * or if the token \#all appears together with any other value.
             */
            XTSE0550,

            /**
             * It is a static error if two parameters of a template or of a
             * stylesheet function have the same name.
             */
            XTSE0580,

            /**
             * It is a static error if a variable-binding element has a select
             * attribute and has non-empty content.
             */
            XTSE0620,

            /**
             * It is a static error if a stylesheet contains more than one binding
             * of a global variable with the same name and same import precedence,
             * unless it also contains another binding with the same name and
             * higher import precedence.
             */
            XTSE0630,

            /**
             * It is a static error if a stylesheet contains an xsl:call-template
             * instruction whose name attribute does not match the name attribute
             * of any xsl:template in the stylesheet.
             */
            XTSE0650,

            /**
             * It is a static error if a stylesheet contains more than one template
             * with the same name and the same import precedence, unless it
             * also contains a template with the same name and higher import
             * precedence.
             */
            XTSE0660,

            /**
             * It is a static error if a single xsl:call-template, xsl:apply-templates,
             * xsl:apply-imports, or xsl:next-match element contains two or
             * more xsl:with-param elements with matching name attributes.
             */
            XTSE0670,

            /**
             * In the case of xsl:call-template, it is a static error to pass
             * a non-tunnel parameter named x to a template that does not have
             * a template parameter named x, unless backwards compatible behavior
             * is enabled for the xsl:call-template instruction.
             */
            XTSE0680,

            /**
             * It is a static error if a template that is invoked using xsl:call-template
             * declares a template parameter specifying required="yes" and not
             * specifying tunnel="yes", if no value for this parameter is supplied
             * by the calling instruction.
             */
            XTSE0690,

            /**
             * It is a static error if the value of the use-attribute-sets attribute
             * of an xsl:copy, xsl:element, or xsl:attribute-set element, or
             * the xsl:use-attribute-sets attribute of a literal result element,
             * is not a whitespace-separated sequence of QNames, or if it contains
             * a QName that does not match the name attribute of any xsl:attribute-set
             * declaration in the stylesheet.
             */
            XTSE0710,

            /**
             * It is a static error if an xsl:attribute-set element directly
             * or indirectly references itself via the names contained in the
             * use-attribute-sets attribute.
             */
            XTSE0720,

            /**
             * A stylesheet function must have a prefixed name, to remove any
             * risk of a clash with a function in the default function namespace.
             * It is a static error if the name has no prefix.
             */
            XTSE0740,

            /**
             * Because arguments to a stylesheet function call must all be specified,
             * the xsl:param elements within an xsl:function element must not
             * specify a default value: this means they must be empty, and must
             * not have a select attribute.
             */
            XTSE0760,

            /**
             * It is a static error for a stylesheet to contain two or more
             * functions with the same expanded-QName, the same arity, and the
             * same import precedence, unless there is another function with
             * the same expanded-QName and arity, and a higher import precedence.
             */
            XTSE0770,

            /**
             * It is a static error if an attribute on a literal result element
             * is in the XSLT namespace, unless it is one of the attributes
             * explicitly defined in this specification.
             */
            XTSE0805,

            /**
             * It is a static error if a namespace prefix is used within the
             * [xsl:]exclude-result-prefixes attribute and there is no namespace
             * binding in scope for that prefix.
             */
            XTSE0808,

            /**
             * It is a static error if the value \#default is used within the
             * [xsl:]exclude-result-prefixes attribute and the parent element
             * of the [xsl:]exclude-result-prefixes attribute has no default
             * namespace.
             */
            XTSE0809,

            /**
             * It is a static error if there is more than one such declaration
             * [more than one xsl:namespace-alias declaration] with the same
             * literal namespace URI and the same import precedence and different
             * values for the target namespace URI, unless there is also an
             * xsl:namespace-alias declaration with the same literal namespace
             * URI and a higher import precedence.
             */
            XTSE0810,

            /**
             * It is a static error if a value other than \#default is specified
             * for either the stylesheet-prefix or the result-prefix attributes
             * of the xsl:namespace-alias element when there is no in-scope
             * binding for that namespace prefix.
             */
            XTSE0812,

            /**
             * It is a static error if the select attribute of the xsl:attribute
             * element is present unless the element has empty content.
             */
            XTSE0840,

            /**
             * It is a static error if the select attribute of the xsl:value-of
             * element is present when the content of the element is non-empty,
             * or if the select attribute is absent when the content is empty.
             */
            XTSE0870,

            /**
             * It is a static error if the select attribute of the xsl:processing-instruction
             * element is present unless the element has empty content.
             */
            XTSE0880,

            /**
             * It is a static error if the select attribute of the xsl:namespace
             * element is present when the element has content other than one
             * or more xsl:fallback instructions, or if the select attribute
             * is absent when the element has empty content.
             */
            XTSE0910,

            /**
             * It is a static error if the select attribute of the xsl:comment
             * element is present unless the element has empty content.
             */
            XTSE0940,

            /**
             * It is a type error to use the xsl:copy or xsl:copy-of instruction
             * to copy a node that has namespace-sensitive content if the copy-namespaces
             * attribute has the value no and its explicit or implicit validation
             * attribute has the value preserve. It is also a type error if
             * either of these instructions (with validation="preserve") is
             * used to copy an attribute having namespace-sensitive content,
             * unless the parent element is also copied. A node has namespace-sensitive
             * content if its typed value contains an item of type xs:QName
             * or xs:NOTATION or a type derived therefrom. The reason this is
             * an error is because the validity of the content depends on the
             * namespace context being preserved.
             */
            XTTE0950,

            /**
             * It is a static error if the value attribute of xsl:number is
             * present unless the select, level, count, and from attributes
             * are all absent.
             */
            XTSE0975,

            /**
             * It is a static error if an xsl:sort element with a select attribute
             * has non-empty content.
             */
            XTSE1015,

            /**
             * It is a static error if an xsl:sort element other than the first
             * in a sequence of sibling xsl:sort elements has a stable attribute.
             */
            XTSE1017,

            /**
             * It is a static error if an xsl:perform-sort instruction with
             * a select attribute has any content other than xsl:sort and xsl:fallback
             * instructions.
             */
            XTSE1040,

            /**
             * It is a static error if the current-group function is used within
             * a pattern.
             */
            XTSE1060,

            /**
             * It is a static error if the current-grouping-key function is
             * used within a pattern.
             */
            XTSE1070,

            /**
             * These four attributes [the group-by, group-adjacent, group-starting-with,
             * and group-ending-with attributes of xsl:for-each-group ] are
             * mutually exclusive: it is a static error if none of these four
             * attributes is present, or if more than one of them is present.
             */
            XTSE1080,

            /**
             * It is an error to specify the collation attribute if neither
             * the group-by attribute nor group-adjacent attribute is specified.
             */
            XTSE1090,

            /**
             * It is a static error if the xsl:analyze-string instruction contains
             * neither an xsl:matching-substring nor an xsl:non-matching-substring
             * element.
             */
            XTSE1130,

            /**
             * It is a static error if an xsl:key declaration has a use attribute
             * and has non-empty content, or if it has empty content and no
             * use attribute.
             */
            XTSE1205,

            /**
             * It is a static error if the xsl:key declaration has a collation
             * attribute whose value (after resolving against the base URI)
             * is not a URI recognized by the implementation as referring to
             * a collation.
             */
            XTSE1210,

            /**
             * It is a static error if there are several xsl:key declarations
             * in the stylesheet with the same key name and different effective
             * collations. Two collations are the same if their URIs are equal
             * under the rules for comparing xs:anyURI values, or if the implementation
             * can determine that they are different URIs referring to the same
             * collation.
             */
            XTSE1220,

            /**
             * It is a static error if a named or unnamed decimal format contains
             * two conflicting values for the same attribute in different xsl:decimal-format
             * declarations having the same import precedence, unless there
             * is another definition of the same attribute with higher import
             * precedence.
             */
            XTSE1290,

            /**
             * It is a static error if the character specified in the zero-digit
             * attribute is not a digit or is a digit that does not have the
             * numeric value zero.
             */
            XTSE1295,

            /**
             * It is a static error if, for any named or unnamed decimal format,
             * the variables representing characters used in a picture string
             * do not each have distinct values. These variables are decimal-separator-sign,
             * grouping-sign, percent-sign, per-mille-sign, digit-zero-sign,
             * digit-sign, and pattern-separator-sign.
             */
            XTSE1300,

            /**
             * It is a static error if there is no namespace bound to the prefix
             * on the element bearing the [xsl:]extension-element-prefixes attribute
             * or, when \#default is specified, if there is no default namespace.
             */
            XTSE1430,

            /**
             * It is a static error if both the [xsl:]type and [xsl:]validation
             * attributes are present on the xsl:element, xsl:attribute, xsl:copy,
             * xsl:copy-of, xsl:document, or xsl:result-document instructions,
             * or on a literal result element.
             */
            XTSE1505,

            /**
             * It is a static error if the value of the type attribute of an
             * xsl:element, xsl:attribute, xsl:copy, xsl:copy-of, xsl:document,
             * or xsl:result-document instruction, or the xsl:type attribute
             * of a literal result element, is not a valid QName, or if it uses
             * a prefix that is not defined in an in-scope namespace declaration,
             * or if the QName is not the name of a type definition included
             * in the in-scope schema components for the stylesheet.
             */
            XTSE1520,

            /**
             * It is a static error if the value of the type attribute of an
             * xsl:attribute instruction refers to a complex type definition
             */
            XTSE1530,

            /**
             * It is a static error if two xsl:output declarations within an
             * output definition specify explicit values for the same attribute
             * (other than cdata-section-elements and use-character-maps), with
             * the values of the attributes being not equal, unless there is
             * another xsl:output declaration within the same output definition
             * that has higher import precedence and that specifies an explicit
             * value for the same attribute.
             */
            XTSE1560,

            /**
             * The value [of the method attribute on xsl:output ] must (if present)
             * be a valid QName. If the QName does not have a prefix, then it
             * identifies a method specified in [XSLT and XQuery Serialization]
             * and must be one of xml, html, xhtml, or text.
             */
            XTSE1570,

            /**
             * It is a static error if the stylesheet contains two or more character
             * maps with the same name and the same import precedence, unless
             * it also contains another character map with the same name and
             * higher import precedence.
             */
            XTSE1580,

            /**
             * It is a static error if a name in the use-character-maps attribute
             * of the xsl:output or xsl:character-map elements does not match
             * the name attribute of any xsl:character-map in the stylesheet.
             */
            XTSE1590,

            /**
             * It is a static error if a character map references itself, directly
             * or indirectly, via a name in the use-character-maps attribute.
             */
            XTSE1600,

            /**
             * A basic XSLT processor must signal a static error if the stylesheet
             * includes an xsl:import-schema declaration.
             */
            XTSE1650,

            /**
             * A basic XSLT processor must signal a static error if the stylesheet
             * includes an [xsl:]type attribute, or an [xsl:]validation or default-validation
             * attribute with a value other than strip.
             */
            XTSE1660,

            /**
             * It is a type error if the result of evaluating the sequence constructor
             * cannot be converted to the required type.
             */
            XTTE0505,

            /**
             * It is a type error if an xsl:apply-templates instruction with
             * no select attribute is evaluated when the context item is not
             * a node.
             */
            XTTE0510,

            /**
             * It is a type error if the sequence returned by the select expression
             * [of xsl:apply-templates] contains an item that is not a node.
             */
            XTTE0520,

            /**
             * It is a type error if the supplied value of a variable cannot
             * be converted to the required type.
             */
            XTTE0570,

            /**
             * It is a type error if the conversion of the supplied value of
             * a parameter to its required type fails.
             */
            XTTE0590,

            /**
             * If a default value is given explicitly, that is, if there is
             * either a select attribute or a non-empty sequence constructor,
             * then it is a type error if the default value cannot be converted
             * to the required type, using the function conversion rules.
             */
            XTTE0600,

            /**
             * If the as attribute [of xsl:function ] is specified, then the
             * result evaluated by the sequence constructor (see 5.7 Sequence
             * Constructors) is converted to the required type, using the function
             * conversion rules. It is a type error if this conversion fails.
             */
            XTTE0780,

            /**
             * If the value of a parameter to a stylesheet function cannot be
             * converted to the required type, a type error is signaled.
             */
            XTTE0790,

            /**
             * It is a type error if the xsl:number instruction is evaluated,
             * with no value or select attribute, when the context item is not
             * a node.
             */
            XTTE0990,

            /**
             * It is a type error if the result of evaluating the select attribute
             * of the xsl:number instruction is anything other than a single
             * node.
             */
            XTTE1000,

            /**
             * If any sort key value, after atomization and any type conversion
             * required by the data-type attribute, is a sequence containing
             * more than one item, then the effect depends on whether the xsl:sort
             * element is evaluated with backwards compatible behavior. With
             * backwards compatible behavior, the effective sort key value is
             * the first item in the sequence. In other cases, this is a type
             * error.
             */
            XTTE1020,

            /**
             * It is a type error if the grouping key evaluated using the group-adjacent
             * attribute is an empty sequence, or a sequence containing more
             * than one item.
             */
            XTTE1100,

            /**
             * When the group-starting-with or group-ending-with attribute [of
             * the xsl:for-each-group instruction] is used, it is a type error
             * if the result of evaluating the select expression contains an
             * item that is not a node.
             */
            XTTE1120,

            /**
             * If the validation attribute of an xsl:element, xsl:attribute,
             * xsl:copy, xsl:copy-of, or xsl:result-document instruction, or
             * the xsl:validation attribute of a literal result element, has
             * the effective value strict, and schema validity assessment concludes
             * that the validity of the element or attribute is invalid or unknown,
             * a type error occurs. As with other type errors, the error may
             * be signaled statically if it can be detected statically.
             */
            XTTE1510,

            /**
             * If the validation attribute of an xsl:element, xsl:attribute,
             * xsl:copy, xsl:copy-of, or xsl:result-document instruction, or
             * the xsl:validation attribute of a literal result element, has
             * the effective value strict, and there is no matching top-level
             * declaration in the schema, then a type error occurs. As with
             * other type errors, the error may be signaled statically if it
             * can be detected statically.
             */
            XTTE1512,

            /**
             * If the validation attribute of an xsl:element, xsl:attribute,
             * xsl:copy, xsl:copy-of, or xsl:result-document instruction, or
             * the xsl:validation attribute of a literal result element, has
             * the effective value lax, and schema validity assessment concludes
             * that the element or attribute is invalid, a type error occurs.
             * As with other type errors, the error may be signaled statically
             * if it can be detected statically.
             */
            XTTE1515,

            /**
             * It is a type error if an [xsl:]type attribute is defined for
             * a constructed element or attribute, and the outcome of schema
             * validity assessment against that type is that the validity property
             * of that element or attribute information item is other than valid.
             */
            XTTE1540,

            /**
             * A type error occurs if a type or validation attribute is defined
             * (explicitly or implicitly) for an instruction that constructs
             * a new attribute node, if the effect of this is to cause the attribute
             * value to be validated against a type that is derived from, or
             * constructed by list or union from, the primitive types xs:QName
             * or xs:NOTATION.
             */
            XTTE1545,

            /**
             * A type error occurs [when a document node is validated] unless
             * the children of the document node comprise exactly one element
             * node, no text nodes, and zero or more comment and processing
             * instruction nodes, in any order.
             */
            XTTE1550,

            /**
             * It is a type error if, when validating a document node, document-level
             * constraints are not satisfied. These constraints include identity
             * constraints (xs:unique, xs:key, and xs:keyref) and ID/IDREF constraints.
             */
            XTTE1555,

            /**
             * It is a non-recoverable dynamic error if the effective value
             * of an attribute written using curly brackets, in a position where
             * an attribute value template is permitted, is a value that is
             * not one of the permitted values for that attribute. If the processor
             * is able to detect the error statically (for example, when any
             * XPath expressions within the curly brackets can be evaluated
             * statically), then the processor may optionally signal this as
             * a static error.
             */
            XTDE0030,

            /**
             * It is a non-recoverable dynamic error if the invocation of the
             * stylesheet specifies a template name that does not match the
             * expanded-QName of a named template defined in the stylesheet.
             */
            XTDE0040,

            /**
             * It is a non-recoverable dynamic error if the invocation of the
             * stylesheet specifies an initial mode (other than the default
             * mode) that does not match the expanded-QName in the mode attribute
             * of any template defined in the stylesheet.
             */
            XTDE0045,

            /**
             * It is a non-recoverable dynamic error if the invocation of the
             * stylesheet specifies both an initial mode and an initial template.
             */
            XTDE0047,

            /**
             * It is a non-recoverable dynamic error if the stylesheet that
             * is invoked declares a visible stylesheet parameter with required="yes"
             * and no value for this parameter is supplied during the invocation
             * of the stylesheet. A stylesheet parameter is visible if it is
             * not masked by another global variable or parameter with the same
             * name and higher import precedence.
             */
            XTDE0050,

            /**
             * It is a non-recoverable dynamic error if the initial template
             * defines a template parameter that specifies required="yes".
             */
            XTDE0060,

            /**
             * If an implementation does not support backwards-compatible behavior,
             * then it is a non-recoverable dynamic error if any element is
             * evaluated that enables backwards-compatible behavior.
             */
            XTDE0160,

            /**
             * It is a recoverable dynamic error if this [the process of finding
             * an xsl:strip-space or xsl:preserve-space declaration to match
             * an element in the source document] leaves more than one match,
             * unless all the matched declarations are equivalent (that is,
             * they are all xsl:strip-space or they are all xsl:preserve-space).
             * Action: The optional recovery action is to select, from the matches
             * that are left, the one that occurs last in declaration order.
             */
            XTRE0270,

            /**
             * Where the result of evaluating an XPath expression (or an attribute
             * value template) is required to be a lexical QName, then unless
             * otherwise specified it is a non-recoverable dynamic error if
             * the defining element has no namespace node whose name matches
             * the prefix of the lexical QName. This error may be signaled as
             * a static error if the value of the expression can be determined
             * statically.
             */
            XTDE0290,

            /**
             * It is a non-recoverable dynamic error if the result sequence
             * used to construct the content of an element node contains a namespace
             * node or attribute node that is preceded in the sequence by a
             * node that is neither a namespace node nor an attribute node.
             */
            XTDE0410,

            /**
             * It is a non-recoverable dynamic error if the result sequence
             * used to construct the content of a document node contains a namespace
             * node or attribute node.
             */
            XTDE0420,

            /**
             * It is a non-recoverable dynamic error if the result sequence
             * contains two or more namespace nodes having the same name but
             * different string values (that is, namespace nodes that map the
             * same prefix to different namespace URIs).
             */
            XTDE0430,

            /**
             * It is a non-recoverable dynamic error if the result sequence
             * contains a namespace node with no name and the element node being
             * constructed has a null namespace URI (that is, it is an error
             * to define a default namespace when the element is in no namespace).
             */
            XTDE0440,

            /**
             * It is a non-recoverable dynamic error if namespace fixup is performed
             * on an element that contains among the typed values of the element
             * and its attributes two values of type xs:QName or xs:NOTATION
             * containing conflicting namespace prefixes, that is, two values
             * that use the same prefix to refer to different namespace URIs.
             */
            XTDE0485,

            /**
             * It is a recoverable dynamic error if the conflict resolution
             * algorithm for template rules leaves more than one matching template
             * rule. Action: The optional recovery action is to select, from
             * the matching template rules that are left, the one that occurs
             * last in declaration order.
             */
            XTRE0540,

            /**
             * It is a non-recoverable dynamic error if xsl:apply-imports or
             * xsl:next-match is evaluated when the current template rule is
             * null.
             */
            XTDE0560,

            /**
             * If an optional parameter has no select attribute and has an empty
             * sequence constructor, and if there is an as attribute, then the
             * default value of the parameter is an empty sequence. If the empty
             * sequence is not a valid instance of the required type defined
             * in the as attribute, then the parameter is treated as a required
             * parameter, which means that it is a non-recoverable dynamic error
             * if the caller supplies no value for the parameter.
             */
            XTDE0610,

            /**
             * In general, a circularity in a stylesheet is a non-recoverable
             * dynamic error.
             */
            XTDE0640,

            /**
             * In other cases, [with xsl:apply-templates, xsl:apply-imports,
             * and xsl:next-match, or xsl:call-template with tunnel parameters]
             * it is a non-recoverable dynamic error if the template that is
             * invoked declares a template parameter with required="yes" and
             * no value for this parameter is supplied by the calling instruction.
             */
            XTDE0700,

            /**
             * It is a recoverable dynamic error if the name of a constructed
             * attribute is xml:space and the value is not either default or
             * preserve. Action: The optional recovery action is to construct
             * the attribute with the value as requested.
             */
            XTRE0795,

            /**
             * It is a non-recoverable dynamic error if the effective value
             * of the name attribute [of the xsl:element instruction] is not
             * a lexical QName.
             */
            XTDE0820,

            /**
             * In the case of an xsl:element instruction with no namespace attribute,
             * it is a non-recoverable dynamic error if the effective value
             * of the name attribute is a QName whose prefix is not declared
             * in an in-scope namespace declaration for the xsl:element instruction.
             */
            XTDE0830,

            /**
             * It is a non-recoverable dynamic error if the effective value
             * of the namespace attribute [of the xsl:element instruction] is
             * not in the lexical space of the xs:anyURI data type.
             */
            XTDE0835,

            /**
             * It is a non-recoverable dynamic error if the effective value
             * of the name attribute [of an xsl:attribute instruction] is not
             * a lexical QName.
             */
            XTDE0850,

            /**
             * In the case of an xsl:attribute instruction with no namespace
             * attribute, it is a non-recoverable dynamic error if the effective
             * value of the name attribute is the string xmlns.
             */
            XTDE0855,

            /**
             * In the case of an xsl:attribute instruction with no namespace
             * attribute, it is a non-recoverable dynamic error if the effective
             * value of the name attribute is a lexical QName whose prefix is
             * not declared in an in-scope namespace declaration for the xsl:attribute
             * instruction.
             */
            XTDE0860,

            /**
             * It is a non-recoverable dynamic error if the effective value
             * of the namespace attribute [of the xsl:attribute instruction]
             * is not in the lexical space of the xs:anyURI data type.
             */
            XTDE0865,

            /**
             * It is a non-recoverable dynamic error if the effective value
             * of the name attribute [of the xsl:processing-instruction instruction]
             * is not both an NCName Names and a PITarget XML.
             */
            XTDE0890,

            /**
             * It is a non-recoverable dynamic error if the string value of
             * the new namespace node [created using xsl:namespace] is not valid
             * in the lexical space of the data type xs:anyURI. [see ERR XTDE0835]
             */
            XTDE0905,

            /**
             * It is a non-recoverable dynamic error if the effective value
             * of the name attribute [of the xsl:namespace instruction] is neither
             * a zero-length string nor an NCName Names, or if it is xmlns.
             */
            XTDE0920,

            /**
             * It is a non-recoverable dynamic error if the xsl:namespace instruction
             * generates a namespace node whose name is xml and whose string
             * value is not http://www.w3.org/XML/1998/namespace, or a namespace
             * node whose string value is http://www.w3.org/XML/1998/namespace
             * and whose name is not xml.
             */
            XTDE0925,

            /**
             * It is a non-recoverable dynamic error if evaluating the select
             * attribute or the contained sequence constructor of an xsl:namespace
             * instruction results in a zero-length string.
             */
            XTDE0930,

            /**
             * It is a non-recoverable dynamic error if any undiscarded item
             * in the atomized sequence supplied as the value of the value attribute
             * of xsl:number cannot be converted to an integer, or if the resulting
             * integer is less than 0 (zero).
             */
            XTDE0980,

            /**
             * It is a non-recoverable dynamic error if, for any sort key component,
             * the set of sort key values evaluated for all the items in the
             * initial sequence, after any type conversion requested, contains
             * a pair of ordinary values for which the result of the XPath lt
             * operator is an error.
             */
            XTDE1030,

            /**
             * It is a non-recoverable dynamic error if the collation attribute
             * of xsl:sort (after resolving against the base URI) is not a URI
             * that is recognized by the implementation as referring to a collation.
             */
            XTDE1035,

            /**
             * It is a non-recoverable dynamic error if the collation URI specified
             * to xsl:for-each-group (after resolving against the base URI)
             * is a collation that is not recognized by the implementation.
             * (For notes, [see ERR XTDE1035].)
             */
            XTDE1110,

            /**
             * It is a non-recoverable dynamic error if the effective value
             * of the regex attribute [of the xsl:analyze-string instruction]
             * does not conform to the required syntax for regular expressions,
             * as specified in [Functions and Operators]. If the regular expression
             * is known statically (for example, if the attribute does not contain
             * any expressions enclosed in curly brackets) then the processor
             * may signal the error as a static error.
             */
            XTDE1140,

            /**
             * It is a non-recoverable dynamic error if the effective value
             * of the flags attribute [of the xsl:analyze-string instruction]
             * has a value other than the values defined in [Functions and Operators].
             * If the value of the attribute is known statically (for example,
             * if the attribute does not contain any expressions enclosed in
             * curly brackets) then the processor may signal the error as a
             * static error.
             */
            XTDE1145,

            /**
             * It is a non-recoverable dynamic error if the effective value
             * of the regex attribute [of the xsl:analyze-string instruction]
             * is a regular expression that matches a zero-length string: or
             * more specifically, if the regular expression $r and flags $f
             * are such that matches("", $r, $f) returns true. If the regular
             * expression is known statically (for example, if the attribute
             * does not contain any expressions enclosed in curly brackets)
             * then the processor may signal the error as a static error.
             */
            XTDE1150,

            /**
             * When a URI reference [supplied to the document function] contains
             * a fragment identifier, it is a recoverable dynamic error if the
             * media type is not one that is recognized by the processor, or
             * if the fragment identifier does not conform to the rules for
             * fragment identifiers for that media type, or if the fragment
             * identifier selects something other than a sequence of nodes (for
             * example, if it selects a range of characters within a text node).
             * Action: The optional recovery action is to ignore the fragment
             * identifier and return the document node.
             */
            XTRE1160,

            /**
             * It is a non-recoverable dynamic error if a URI [supplied in the
             * first argument to the unparsed-text function] contains a fragment
             * identifier, or if it cannot be used to retrieve a resource containing
             * text.
             */
            XTDE1170,

            /**
             * It is a non-recoverable dynamic error if a resource [retrieved
             * using the unparsed-text function] contains octets that cannot
             * be decoded into Unicode characters using the specified encoding,
             * or if the resulting characters are not permitted XML characters.
             * This includes the case where the processor does not support the
             * requested encoding.
             */
            XTDE1190,

            /**
             * It is a non-recoverable dynamic error if the second argument
             * of the unparsed-text function is omitted and the processor cannot
             * infer the encoding using external information and the encoding
             * is not UTF-8.
             */
            XTDE1200,

            /**
             * It is a non-recoverable dynamic error if the value [of the first
             * argument to the key function] is not a valid QName, or if there
             * is no namespace declaration in scope for the prefix of the QName,
             * or if the name obtained by expanding the QName is not the same
             * as the expanded name of any xsl:key declaration in the stylesheet.
             * If the processor is able to detect the error statically (for
             * example, when the argument is supplied as a string literal),
             * then the processor may optionally signal this as a static error.
             */
            XTDE1260,

            /**
             * It is a non-recoverable dynamic error to call the key function
             * with two arguments if there is no context node, or if the root
             * of the tree containing the context node is not a document node;
             * or to call the function with three arguments if the root of the
             * tree containing the node supplied in the third argument is not
             * a document node.
             */
            XTDE1270,

            /**
             * It is a non-recoverable dynamic error if the name specified as
             * the $decimal-format-name argument [ to the format-number function]
             * is not a valid QName, or if its prefix has not been declared
             * in an in-scope namespace declaration, or if the stylesheet does
             * not contain a declaration of a decimal-format with a matching
             * expanded-QName. If the processor is able to detect the error
             * statically (for example, when the argument is supplied as a string
             * literal), then the processor may optionally signal this as a
             * static error.
             */
            XTDE1280,

            /**
             * The picture string [supplied to the format-number function] must
             * conform to the following rules. [ See full specification.] It
             * is a non-recoverable dynamic error if the picture string does
             * not satisfy these rules.
             */
            XTDE1310,

            /**
             * It is a non-recoverable dynamic error if the syntax of the picture
             * [used for date/time formatting] is incorrect.
             */
            XTDE1340,

            /**
             * It is a non-recoverable dynamic error if a component specifier
             * within the picture [used for date/time formatting] refers to
             * components that are not available in the given type of $value,
             * for example if the picture supplied to the format-time refers
             * to the year, month, or day component.
             */
            XTDE1350,

            /**
             * If the current function is evaluated within an expression that
             * is evaluated when the context item is undefined, a non-recoverable
             * dynamic error occurs.
             */
            XTDE1360,

            /**
             * It is a non-recoverable dynamic error if the unparsed-entity-uri
             * function is called when there is no context node, or when the
             * root of the tree containing the context node is not a document
             * node.
             */
            XTDE1370,

            /**
             * It is a non-recoverable dynamic error if the unparsed-entity-public-id
             * function is called when there is no context node, or when the
             * root of the tree containing the context node is not a document
             * node.
             */
            XTDE1380,

            /**
             * It is a non-recoverable dynamic error if the value [supplied
             * as the $property-name argument to the system-property function]
             * is not a valid QName, or if there is no namespace declaration
             * in scope for the prefix of the QName. If the processor is able
             * to detect the error statically (for example, when the argument
             * is supplied as a string literal), then the processor may optionally
             * signal this as a static error.
             */
            XTDE1390,

            /**
             * When a transformation is terminated by use of xsl:message terminate="yes",
             * the effect is the same as when a non-recoverable dynamic error
             * occurs during the transformation.
             */
            XTMM9000,

            /**
             * It is a non-recoverable dynamic error if the argument [passed
             * to the function-available function] does not evaluate to a string
             * that is a valid QName, or if there is no namespace declaration
             * in scope for the prefix of the QName. If the processor is able
             * to detect the error statically (for example, when the argument
             * is supplied as a string literal), then the processor may optionally
             * signal this as a static error.
             */
            XTDE1400,

            /**
             * It is a non-recoverable dynamic error if the arguments supplied
             * to a call on an extension function do not satisfy the rules defined
             * for that particular extension function, or if the extension function
             * reports an error, or if the result of the extension function
             * cannot be converted to an XPath value.
             */
            XTDE1420,

            /**
             * When backwards compatible behavior is enabled, it is a non-recoverable
             * dynamic error to evaluate an extension function call if no implementation
             * of the extension function is available.
             */
            XTDE1425,

            /**
             * It is a non-recoverable dynamic error if the argument [passed
             * to the type-available function] does not evaluate to a string
             * that is a valid QName, or if there is no namespace declaration
             * in scope for the prefix of the QName. If the processor is able
             * to detect the error statically (for example, when the argument
             * is supplied as a string literal), then the processor may optionally
             * signal this as a static error.
             */
            XTDE1428,

            /**
             * It is a non-recoverable dynamic error if the argument [passed
             * to the element-available function] does not evaluate to a string
             * that is a valid QName, or if there is no namespace declaration
             * in scope for the prefix of the QName. If the processor is able
             * to detect the error statically (for example, when the argument
             * is supplied as a string literal), then the processor may optionally
             * signal this as a static error.
             */
            XTDE1440,

            /**
             * When a processor performs fallback for an extension instruction
             * that is not recognized, if the instruction element has one or
             * more xsl:fallback children, then the content of each of the xsl:fallback
             * children must be evaluated; it is a non-recoverable dynamic error
             * if it has no xsl:fallback children.
             */
            XTDE1450,

            /**
             * It is a non-recoverable dynamic error if the effective value
             * of the format attribute [of an xsl:result-document element] is
             * not a valid lexical QName, or if it does not match the expanded-QName
             * of an output definition in the stylesheet. If the processor is
             * able to detect the error statically (for example, when the format
             * attribute contains no curly brackets), then the processor may
             * optionally signal this as a static error.
             */
            XTDE1460,

            /**
             * It is a non-recoverable dynamic error to evaluate the xsl:result-document
             * instruction in temporary output state.
             */
            XTDE1480,

            /**
             * It is a non-recoverable dynamic error for a transformation to
             * generate two or more final result trees with the same URI.
             */
            XTDE1490,

            /**
             * It is a recoverable dynamic error for a transformation to generate
             * two or more final result trees with URIs that identify the same
             * physical resource. The optional recovery action is implementation-dependent,
             * since it may be impossible for the processor to detect the error.
             */
            XTRE1495,

            /**
             * It is a recoverable dynamic error for a stylesheet to write to
             * an external resource and read from the same resource during a
             * single transformation, whether or not the same URI is used to
             * access the resource in both cases. Action: The optional recovery
             * action is implementation-dependent: implementations are not required
             * to detect the error condition. Note that if the error is not
             * detected, it is undefined whether the document that is read from
             * the resource reflects its state before or after the result tree
             * is written.
             */
            XTRE1500,

            /**
             * It is a recoverable dynamic error if an xsl:value-of or xsl:text
             * instruction specifies that output escaping is to be disabled
             * and the implementation does not support this. Action: The optional
             * recovery action is to ignore the disable-output-escaping attribute.
             */
            XTRE1620,

            /**
             * It is a recoverable dynamic error if an xsl:value-of or xsl:text
             * instruction specifies that output escaping is to be disabled
             * when writing to a final result tree that is not being serialized.
             * Action: The optional recovery action is to ignore the disable-output-escaping
             * attribute.
             */
            XTRE1630,

            /**
             * A basic XSLT processor must raise a non-recoverable dynamic error
             * if the input to the processor includes a node with a type annotation
             * other than xs:untyped or xs:untypedAtomic, or an atomic value
             * of a type other than those which a basic XSLT processor supports.
             */
            XTDE1665

        };

        /**
         * Issues a warning, should not be used excessively. This can
         * be used to communicate that a certain implementation defined
         * feature is unsupported or that a certain expression most likely
         * doesn't do what the users wants, to name a few examples.
         *
         * @see <a href="http://www.w3.org/TR/xpath20/#errors">XML Path Language (XPath) 2.0,
         * 2.3 Error Handling</a>
         * @param message the message to be read by the user.
         * @param sourceLocation the location of where the warning originates from.
         */
        void warning(const QString &message, const QSourceLocation &sourceLocation = QSourceLocation());

        /**
         * Issues an error. May be used at the static analysis phase or
         * the dynamic evaluation phase.
         *
         * For SourceLocationReflection instances, the overload taking an SouourceLocationReflection should be used.
         *
         * @see <a href="http://www.w3.org/TR/xpath20/#errors">XML Path Language (XPath) 2.0,
         * 2.3 Error Handling</a>
         * @param message the message to be read by the user.
         * @param errorCode identifies the error condition, as described
         * @param sourceLocation the location of where the error originates from
         * in "XML Path Language (XPath) 2.0" section "G Error Conditions"
         */
        void error(const QString &message,
                   const ReportContext::ErrorCode errorCode,
                   const QSourceLocation &sourceLocation);

        /**
         * Overload.
         *
         * Same as the above, but passes the SourceLocationReflection as reference for error reporting.
         */
        void error(const QString &message,
                   const ReportContext::ErrorCode errorCode,
                   const SourceLocationReflection *const reflection);

        /**
         * Issues an error which is not identified in the XPath specifications. This function
         * is among other things used for implementing the <tt>fn:error()</tt> function.
         */
        void error(const QString &message,
                   const QXmlName qName,
                   const SourceLocationReflection *const r);

        /**
         * @return the QAbstractMessageHandler which functions such as warning() and
         * error() should submit messages to. This function
         * may never return @c null; a valid QAbstractMessageHandler pointer must always be returned.
         */
        virtual QAbstractMessageHandler *messageHandler() const = 0;

        virtual NamePool::Ptr namePool() const = 0;

        /**
         * Returns a string representation of the error code @p code.
         *
         * @see ReportContext::ErrorCode
         * @param errorCode identifies the error condition, as described
         * in <a href="http://www.w3.org/TR/xpath20/#id-errors">XML Path
         * Language (XPath) 2.0, G Error Conditions</a>
         */
        static QString codeToString(const ReportContext::ErrorCode errorCode);

        /**
         * @returns the error code part of @p typeURI and sets @p uri to the error namespace. Note
         * that the error namespace not necessarily is the namespace for XPath and
         * XQuery errors, http://www.w3.org/2005/xqt-errors, but can be user defined.
         */
        static QString codeFromURI(const QString &typeURI,
                                   QString &uri);

        /**
         * @short Returns the source location applying for @p reflection.
         */
        virtual QSourceLocation locationFor(const SourceLocationReflection *const reflection) const = 0;

        /**
         * Resolves @p relative against @p baseURI, possibly using a URI resolver.
         */
        QUrl resolveURI(const QUrl &relative,
                        const QUrl &baseURI) const;

        /**
         * @short The URI resolver in use.
         *
         * If no URI resolver is in use, a @c null pointer is returned.
         *
         * @note You should probably use resolveURI(), which handles the case of
         * when uriResolver() is @c null.
         */
        virtual const QAbstractUriResolver *uriResolver() const = 0;

    private:
        void createError(const QString &description,
                         const QtMsgType type,
                         const QUrl &id,
                         const QSourceLocation &sourceLocation) const;
        static inline QString finalizeDescription(const QString &desc);
        QSourceLocation lookupSourceLocation(const SourceLocationReflection *const ref) const;

        Q_DISABLE_COPY(ReportContext)
    };

    /**
     * @short This is the class type that is being thrown when a query error occur.
     *
     * @relates ReportContext
     */
    typedef bool Exception;
}

QT_END_NAMESPACE

#endif
