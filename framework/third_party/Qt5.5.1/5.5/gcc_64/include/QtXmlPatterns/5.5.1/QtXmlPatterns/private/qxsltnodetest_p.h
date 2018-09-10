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

#ifndef Patternist_XSLTNodeTest_H
#define Patternist_XSLTNodeTest_H

#include <private/qanynodetype_p.h>

QT_BEGIN_NAMESPACE

namespace QPatternist
{

    /**
     * @short Represents @c node() in patterns in XSL-T, which
     * are just like how @c node() usually is, except that it doesn't
     * match document nodes.
     *
     * @see <a href="http://www.w3.org/TR/xslt20/#pattern-semantics">XSL
     * Transformations (XSLT) Version 2.0, 5.5.3 The Meaning of a Pattern</a>
     * @ingroup Patternist_types
     * @author Frans Englich <frans.englich@nokia.com>
     * @since 4.5
     */
    class XSLTNodeTest : public AnyNodeType
    {
    public:
        virtual bool xdtTypeMatches(const ItemType::Ptr &other) const;
        virtual bool itemMatches(const Item &item) const;

        virtual ItemType::Ptr xdtSuperType() const;

    protected:
        friend class BuiltinTypes;

        /**
         * This constructor does nothing, but exists in order to make it impossible to
         * instantiate this class from anywhere but from BuiltinTypes.
         */
        inline XSLTNodeTest()
        {
        }
    };
}

QT_END_NAMESPACE

#endif
