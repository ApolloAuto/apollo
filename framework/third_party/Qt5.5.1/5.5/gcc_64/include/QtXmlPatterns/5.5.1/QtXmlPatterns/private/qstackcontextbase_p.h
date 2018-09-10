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

#ifndef Patternist_StackContextBase_H
#define Patternist_StackContextBase_H

#include <QVector>

#include <private/qdaytimeduration_p.h>
#include <private/qdelegatingdynamiccontext_p.h>
#include <private/qexpression_p.h>

QT_BEGIN_NAMESPACE

namespace QPatternist
{
    /**
     * @short Base class for all DynamicContext classes that needs to supply
     * variables. It has a new frame for local caches, position iterators,
     * expressions, range variables, template parameters but notably continues
     * to delegate global caches.
     *
     * @author Frans Englich <frans.englich@nokia.com>
     */
    template<typename TSuperClass>
    class StackContextBase : public TSuperClass
    {
    public:
        StackContextBase();
        /**
         * Construct a StackContextBase and passes @p prevContext to its super class. This
         * constructor is typically used when the super class is DelegatingDynamicContext.
         */
        StackContextBase(const DynamicContext::Ptr &prevContext);

        virtual void setRangeVariable(const VariableSlotID slotNumber,
                                      const Item &newValue);
        virtual Item rangeVariable(const VariableSlotID slotNumber) const;

        virtual void setExpressionVariable(const VariableSlotID slotNumber,
                                           const Expression::Ptr &newValue);
        virtual Expression::Ptr expressionVariable(const VariableSlotID slotNumber) const;

        virtual Item::Iterator::Ptr positionIterator(const VariableSlotID slot) const;
        virtual void setPositionIterator(const VariableSlotID slot,
                                         const Item::Iterator::Ptr &newValue);
        virtual ItemCacheCell &itemCacheCell(const VariableSlotID slot);
        virtual ItemSequenceCacheCell::Vector &itemSequenceCacheCells(const VariableSlotID slot);

        virtual DynamicContext::TemplateParameterHash &templateParameterStore();

    protected:
        /**
         * This function is protected, although it only is used in this class. I don't
         * know why it has to be, but it won't compile when private.
         */
        template<typename VectorType, typename UnitType>
        inline
        void setSlotVariable(const VariableSlotID slot,
                             const UnitType &newValue,
                             VectorType &container) const;

    private:
        Item::Vector                            m_rangeVariables;
        Expression::Vector                      m_expressionVariables;
        Item::Iterator::Vector                  m_positionIterators;
        ItemCacheCell::Vector                   m_itemCacheCells;
        ItemSequenceCacheCell::Vector           m_itemSequenceCacheCells;
        DynamicContext::TemplateParameterHash   m_templateParameterStore;
    };

    #include "qstackcontextbase_tpl_p.h"

    /**
     * @short A DynamicContext that creates a new scope for variables.
     *
     * This DynamicContext is used for recursive user function calls, for example.
     */
    typedef StackContextBase<DelegatingDynamicContext> StackContext;
}

QT_END_NAMESPACE

#endif
