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

#ifndef Patternist_PatternPlatform_H
#define Patternist_PatternPlatform_H

#include <QFlags>
#include <QRegExp>

#include <private/qfunctioncall_p.h>

QT_BEGIN_NAMESPACE

namespace QPatternist
{
    /**
     * @short Contains functionality for functions and expressions that
     * uses regular expressions.
     *
     * @ingroup Patternist_utils
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class PatternPlatform : public FunctionCall
    {
    public:
        /**
         * @see <a href="http://www.w3.org/TR/xpath-functions/#flags">XQuery 1.0 and
         * XPath 2.0 Functions and Operators, 7.6.1.1 Flags</a>
         */
        enum Flag
        {
            /**
             * No flags are set. Default behavior is used.
             */
            NoFlags             = 0,

            /**
             * Flag @c s
             */
            DotAllMode          = 1,

            /**
             * Flag @c m
             */
            MultiLineMode       = 2,

            /**
             * Flag @c i
             */
            CaseInsensitive     = 4,

            /**
             * Flag @c x
             */
            SimplifyWhitespace  = 8
        };
        typedef QFlags<Flag> Flags;

        virtual Expression::Ptr compress(const StaticContext::Ptr &context);

        /**
         * Retrieves the pattern supplied in the arguments, taking care of compiling it,
         * settings its flags, and everything else required for getting it ready to use. If an error
         * occurs, an appropriate error is raised via @p context.
         */
        QRegExp pattern(const DynamicContext::Ptr &context) const;

        /**
         * @returns the number of captures, also called parenthesized sub-expressions, the pattern has.
         *
         * If the pattern isn't precompiled, -1 is returned.
         */
        inline int captureCount() const;

        /**
         * @short Parses pattern
         */
        static QRegExp parsePattern(const QString &pattern,
                                    const ReportContext::Ptr &context,
                                    const SourceLocationReflection *const location);


    protected:
        /**
         * @short This constructor is protected, because this class is supposed to be sub-classed.
         *
         * @param flagsPosition an index position specifying the operand containing the pattern
         * flags.
         */
        PatternPlatform(const qint8 flagsPosition);

    private:
        /**
         * Enum telling whether the flags, pattern, or both
         * have been compiled at compile time.
         */
        enum PreCompiledPart
        {
            NoPart          = 0,
            PatternPrecompiled     = 1,
            FlagsPrecompiled       = 2,
            FlagsAndPattern = PatternPrecompiled | FlagsPrecompiled

        };
        typedef QFlags<PreCompiledPart> PreCompiledParts;

        /**
         * @short Calls the public parsePattern() function and passes in @c
         * this as the location.
         */
        inline QRegExp parsePattern(const QString &pattern,
                                    const ReportContext::Ptr &context) const;

        Q_DISABLE_COPY(PatternPlatform)

        Flags parseFlags(const QString &flags,
                         const DynamicContext::Ptr &context) const;

        static void applyFlags(const Flags flags, QRegExp &pattern);

        /**
         * The parts that have been pre-compiled at compile time.
         */
        PreCompiledParts    m_compiledParts;
        Flags               m_flags;
        QRegExp             m_pattern;
        const qint8         m_flagsPosition;
    };

    inline int PatternPlatform::captureCount() const
    {
        if(m_compiledParts.testFlag(PatternPrecompiled))
            return m_pattern.captureCount();
        else
            return -1;
    }

    Q_DECLARE_OPERATORS_FOR_FLAGS(PatternPlatform::Flags)
}

QT_END_NAMESPACE

#endif
