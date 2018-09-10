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

#ifndef Patternist_SourceLocationReflection_H
#define Patternist_SourceLocationReflection_H

QT_BEGIN_NAMESPACE

class QString;

namespace QPatternist
{
    /**
     * @short Base class for all instances that represents something
     * at a certain location.
     *
     * SourceLocationReflection does not provide the source location itself,
     * the address to an instance is the mark for it, that in turn can be used
     * for looking up the source location where that mapping is provided.
     *
     * However, this SourceLocationReflection is not itself the mark. The real
     * mark is retrieved by calling actualReflection(). This mechanism
     * allows a SourceLocationReflection sub-class to delegate, or be an alias,
     * for another source location mark.
     *
     * If sourceLocation() returns a non-null object, it will be used instead
     * of looking up via actualReflection().
     *
     * @author Frans Englich <frans.englich@nokia.com>
     * @ingroup Patternist_expressions
     */
    class Q_AUTOTEST_EXPORT SourceLocationReflection
    {
    public:
        inline SourceLocationReflection()
        {
        }

        virtual ~SourceLocationReflection()
        {
        }

        virtual const SourceLocationReflection *actualReflection() const = 0;

        /**
         * A description of what represents the source code location, for
         * human consumption. Must be translated, as appropriate.
         */
        virtual QString description() const
        {
            return QString();
        }

        virtual QSourceLocation sourceLocation() const;

    private:
        Q_DISABLE_COPY(SourceLocationReflection)
    };

    class DelegatingSourceLocationReflection : public SourceLocationReflection
    {
    public:
        inline DelegatingSourceLocationReflection(const SourceLocationReflection *const r) : m_r(r)
        {
            Q_ASSERT(r);
        }

        virtual const SourceLocationReflection *actualReflection() const;
        virtual QString description() const;

    private:
        const SourceLocationReflection *const m_r;
    };

}

QT_END_NAMESPACE

#endif
