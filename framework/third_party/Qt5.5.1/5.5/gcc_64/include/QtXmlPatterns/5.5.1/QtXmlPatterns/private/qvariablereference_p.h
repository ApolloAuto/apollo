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

#ifndef Patternist_VariableReference_H
#define Patternist_VariableReference_H

#include <private/qemptycontainer_p.h>

QT_BEGIN_NAMESPACE

template<typename T> class QList;

namespace QPatternist
{
    /**
     * @short Baseclass for classes being references to variables.
     *
     * @author Frans Englich <frans.englich@nokia.com>
     * @ingroup Patternist_expressions
     */
    class VariableReference : public EmptyContainer
    {
    public:
        typedef QExplicitlySharedDataPointer<VariableReference> Ptr;
        typedef QList<VariableReference::Ptr> List;

        /**
         * Creates a VariableReference.
         *
         * @param slot must be a valid slot. That is, zero or larger.
         */
        VariableReference(const VariableSlotID slot);

        /**
         * @returns the slot that this reference communicates through.
         *
         * This is a slot in the DynamicContext. Which one, depends on the
         * type, which this VariableReference does not have information about.
         * For instance, it could DynamicContext::expressionVariable() or
         * DynamicContext::rangeVariable().
         */
        inline VariableSlotID slot() const;

        /**
         * @returns DisableElimination
         */
        virtual Properties properties() const;

    private:
        /**
         * The slot. Same as returned by slot().
         *
         *  This variable is not called m_slot, because that creates a weird
         *  compiler error on hpuxi-acc. See the preprocessor output. EvaluationCache::m_varSlot
         *  is a similar workaround.
         */
        const VariableSlotID m_varSlot;
    };

    inline VariableSlotID VariableReference::slot() const
    {
        return m_varSlot;
    }

}

QT_END_NAMESPACE

#endif
