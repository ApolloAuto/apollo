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

#ifndef Patternist_XsdSchemaTypesFactory_H
#define Patternist_XsdSchemaTypesFactory_H

#include <QtCore/QHash>
#include <private/qschematypefactory_p.h>

QT_BEGIN_NAMESPACE

namespace QPatternist
{

    /**
     * @short Factory for creating schema types for the types defined in XSD.
     *
     * @ingroup Patternist_types
     * @author Tobias Koenig <tobias.koenig@nokia.com>
     */
    class XsdSchemaTypesFactory : public SchemaTypeFactory
    {
        public:
            /**
             * Creates a new schema type factory.
             *
             * @param namePool The name pool all type names belong to.
             */
            XsdSchemaTypesFactory(const NamePool::Ptr &namePool);

            /**
             * Creates a primitive type for @p name. If @p name is not supported,
             * @c null is returned.
             *
             * @note This does not handle user defined types, only builtin types.
             */
            virtual SchemaType::Ptr createSchemaType(const QXmlName) const;

            /**
             * Returns a hash of all available types.
             */
            virtual SchemaType::Hash types() const;

        private:
            /**
             * A dictonary of all predefined schema types.
             */
            SchemaType::Hash               m_types;

            NamePool::Ptr                  m_namePool;
            mutable SchemaTypeFactory::Ptr m_basicTypesFactory;
    };
}

QT_END_NAMESPACE

#endif
