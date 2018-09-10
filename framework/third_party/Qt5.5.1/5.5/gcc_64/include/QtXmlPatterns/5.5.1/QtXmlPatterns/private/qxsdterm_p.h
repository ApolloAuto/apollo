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

#ifndef Patternist_XsdTerm_H
#define Patternist_XsdTerm_H

#include <private/qnamedschemacomponent_p.h>
#include <private/qxsdannotated_p.h>

QT_BEGIN_NAMESPACE

namespace QPatternist
{
    /**
     * @short A base class for all particles of a model group.
     *
     * This class is the base class for all particles of a model group
     * as the <em>element</em>, <em>group</em> or <em>any</em> tag, it is not supposed to
     * be instantiated directly.
     *
     * @see <a href="http://www.w3.org/Submission/2004/SUBM-xmlschema-api-20040309/xml-schema-api.html#Interface-XSTerm">XML Schema API reference</a>
     * @ingroup Patternist_schema
     * @author Tobias Koenig <tobias.koenig@nokia.com>
     */
    class XsdTerm : public NamedSchemaComponent, public XsdAnnotated
    {
        public:
            typedef QExplicitlySharedDataPointer<XsdTerm> Ptr;

            /**
             * Returns @c true if the term is an element, @c false otherwise.
             */
            virtual bool isElement() const;

            /**
             * Returns @c true if the term is a model group (group tag), @c false otherwise.
             */
            virtual bool isModelGroup() const;

            /**
             * Returns @c true if the term is a wildcard (any tag), @c false otherwise.
             */
            virtual bool isWildcard() const;

            /**
             * Returns @c true if the term is a reference, @c false otherwise.
             *
             * @note The reference term is only used internally as helper during type resolving.
             */
            virtual bool isReference() const;

        protected:
            /**
             * This constructor only exists to ensure this class is subclassed.
             */
            inline XsdTerm() {};
    };
}

QT_END_NAMESPACE

#endif
