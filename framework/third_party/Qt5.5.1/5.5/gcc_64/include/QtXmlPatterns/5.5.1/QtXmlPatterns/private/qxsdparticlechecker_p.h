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

#ifndef Patternist_XsdParticleChecker_H
#define Patternist_XsdParticleChecker_H

#include <private/qxsdelement_p.h>
#include <private/qxsdparticle_p.h>
#include <private/qxsdschemacontext_p.h>
#include <private/qxsdwildcard_p.h>

QT_BEGIN_NAMESPACE

namespace QPatternist
{
    /**
     * @short A helper class to check validity of particles.
     *
     * @ingroup Patternist_schema
     * @author Tobias Koenig <tobias.koenig@nokia.com>
     */
    class XsdParticleChecker
    {
        public:
            /**
             * Checks whether the given @p particle has two or more element
             * declarations with the same name but different type definitions.
             */
            static bool hasDuplicatedElements(const XsdParticle::Ptr &particle, const NamePool::Ptr &namePool, XsdElement::Ptr &conflictingElement);

            /**
             * Checks whether the given @p particle is valid according the
             * UPA (http://www.w3.org/TR/xmlschema-1/#cos-nonambig) constraint.
             */
            static bool isUPAConform(const XsdParticle::Ptr &particle, const NamePool::Ptr &namePool);

            /**
             * Checks whether the given @p particle, which must be an xsd:all element,
             * is valid according the UPA (http://www.w3.org/TR/xmlschema-1/#cos-nonambig) constraint.
             * For xsd:all elements, we do not want to construct a state machine.
             */
            static bool isUPAConformXsdAll(const XsdParticle::Ptr &particle, const NamePool::Ptr &namePool);

            /**
             * Checks whether the given @p particle subsumes the given @p derivedParticle.
             * (http://www.w3.org/TR/xmlschema-1/#cos-particle-restrict)
             */
            static bool subsumes(const XsdParticle::Ptr &particle, const XsdParticle::Ptr &derivedParticle, const XsdSchemaContext::Ptr &context, QString &errorMsg);
    };
}

QT_END_NAMESPACE

#endif
