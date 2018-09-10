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

#ifndef Patternist_XsdUserSchemaType_H
#define Patternist_XsdUserSchemaType_H

#include <private/qnamedschemacomponent_p.h>
#include <private/qschematype_p.h>
#include <private/qxsdannotated_p.h>

QT_BEGIN_NAMESPACE

template<typename N, typename M> class QHash;
template<typename N> class QList;

namespace QPatternist
{
    /**
     * @short A base class for all user defined simple and complex types.
     *
     * This class was introduced to combine the SchemaType class and the
     * NamedSchemaComponent class without explicit inheritance.
     *
     * @ingroup Patternist_schema
     * @author Tobias Koenig <tobias.koenig@nokia.com>
     */
    template<typename TSuperClass>
    class XsdUserSchemaType : public TSuperClass, public NamedSchemaComponent, public XsdAnnotated
    {
        public:
            typedef QExplicitlySharedDataPointer<XsdUserSchemaType> Ptr;

            /**
             * Sets the @p name of the type.
             */
            void setName(const QXmlName &name);

            /**
             * Returns the name of the type.
             *
             * @param namePool The pool the name belongs to.
             */
            virtual QXmlName name(const NamePool::Ptr &namePool) const;

            /**
             * Returns the display name of the type.
             *
             * @param namePool The pool the name belongs to.
             */
            virtual QString displayName(const NamePool::Ptr &namePool) const;

            /**
             * Sets the derivation @p constraints of the type.
             */
            void setDerivationConstraints(const SchemaType::DerivationConstraints &constraints);

            /**
             * Returns the derivation constraints of the type.
             */
            SchemaType::DerivationConstraints derivationConstraints() const;

        private:
            QXmlName                          m_name;
            SchemaType::DerivationConstraints m_derivationConstraints;
    };

    #include "qxsduserschematype_tpl_p.h"
}

QT_END_NAMESPACE

#endif
