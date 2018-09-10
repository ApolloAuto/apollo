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

#ifndef Patternist_XsdSchemaDebugger_H
#define Patternist_XsdSchemaDebugger_H

#include <private/qxsdschema_p.h>

QT_BEGIN_NAMESPACE

namespace QPatternist
{
    /**
     * A helper class to print out the structure of a compiled schema.
     */
    class XsdSchemaDebugger
    {
        public:
            /**
             * Creates a new schema debugger.
             *
             * @param namePool The name pool that the schema uses.
             */
            XsdSchemaDebugger(const NamePool::Ptr &namePool);

            /**
             * Dumps the structure of the given @p particle.
             *
             * @param particle The particle to dump.
             * @param level The level of indention.
             */
            void dumpParticle(const XsdParticle::Ptr &particle, int level = 0);

            /**
             * Dumps the inheritance path of the given @p type.
             *
             * @param type The type to dump.
             * @param level The level of indention.
             */
            void dumpInheritance(const SchemaType::Ptr &type, int level = 0);

            /**
             * Dumps the structure of the given @p wildcard.
             */
            void dumpWildcard(const XsdWildcard::Ptr &wildcard);

            /**
             * Dumps the structure of the given @p type.
             */
            void dumpType(const SchemaType::Ptr &type);

            /**
             * Dumps the structure of the given @p element.
             */
            void dumpElement(const XsdElement::Ptr &element);

            /**
             * Dumps the structure of the given @p attribute.
             */
            void dumpAttribute(const XsdAttribute::Ptr &attribute);

            /**
             * Dumps the structure of the complete @p schema.
             */
            void dumpSchema(const XsdSchema::Ptr &schema);

        private:
            const NamePool::Ptr m_namePool;
    };

}

QT_END_NAMESPACE

#endif
