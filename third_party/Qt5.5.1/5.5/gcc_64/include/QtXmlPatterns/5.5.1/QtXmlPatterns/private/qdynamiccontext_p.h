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

#ifndef Patternist_DynamicContext_H
#define Patternist_DynamicContext_H

#include <private/qautoptr_p.h>
#include <private/qcachecells_p.h>
#include <private/qexternalvariableloader_p.h>
#include <private/qitem_p.h>
#include <private/qnamepool_p.h>
#include <private/qnodebuilder_p.h>
#include <private/qprimitives_p.h>
#include <private/qreportcontext_p.h>
#include <private/qresourceloader_p.h>

QT_BEGIN_NAMESPACE

class QDateTime;
template<typename T> class QVector;

namespace QPatternist
{
    class DayTimeDuration;
    class Expression;
    class TemplateMode;

    /**
     * @short Carries information and facilities used at runtime, and hence
     * provides a state for that stage in a thread-safe manner.
     *
     * @see <a href="http://www.w3.org/TR/xquery/#eval_context">XQuery
     * 1.0: An XML Query Language, 2.1.2 Dynamic Context</a>
     * @see <a href="http://www.w3.org/TR/xquery/#id-dynamic-evaluation">XQuery
     * 1.0: An XML Query Language, 2.2.3.2 Dynamic Evaluation Phase</a>
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class DynamicContext : public ReportContext
    {
    public:
        /**
         * @short Carries template parameters at runtime.
         *
         * The key is the name of the parameter, and the value the Expression
         * which supplies the value.
         */
        typedef QHash<QXmlName, QExplicitlySharedDataPointer<Expression> > TemplateParameterHash;
        typedef QExplicitlySharedDataPointer<DynamicContext> Ptr;

        virtual ~DynamicContext()
        {
        }

        /**
         * This function intentionally returns by reference.
         *
         * @see globalItemCacheCell()
         */
        virtual ItemCacheCell &itemCacheCell(const VariableSlotID slot) = 0;

        /**
         * This function intentionally returns by reference.
         *
         * @see globalItemSequenceCacheCells
         */
        virtual ItemSequenceCacheCell::Vector &itemSequenceCacheCells(const VariableSlotID slot) = 0;

        virtual xsInteger contextPosition() const = 0;
        virtual Item contextItem() const = 0;
        virtual xsInteger contextSize() = 0;

        virtual void setRangeVariable(const VariableSlotID slot,
                                      const Item &newValue) = 0;
        virtual Item rangeVariable(const VariableSlotID slot) const = 0;
        virtual void setExpressionVariable(const VariableSlotID slot,
                                           const QExplicitlySharedDataPointer<Expression> &newValue) = 0;
        virtual QExplicitlySharedDataPointer<Expression>
        expressionVariable(const VariableSlotID slot) const = 0;

        virtual Item::Iterator::Ptr positionIterator(const VariableSlotID slot) const = 0;
        virtual void setPositionIterator(const VariableSlotID slot,
                                         const Item::Iterator::Ptr &newValue) = 0;

        virtual void setFocusIterator(const Item::Iterator::Ptr &it) = 0;
        virtual Item::Iterator::Ptr focusIterator() const = 0;

        virtual QExplicitlySharedDataPointer<DayTimeDuration> implicitTimezone() const = 0;
        virtual QDateTime currentDateTime() const = 0;

        virtual QAbstractXmlReceiver *outputReceiver() const = 0;
        virtual NodeBuilder::Ptr nodeBuilder(const QUrl &baseURI) const = 0;
        virtual ResourceLoader::Ptr resourceLoader() const = 0;
        virtual ExternalVariableLoader::Ptr externalVariableLoader() const = 0;
        virtual NamePool::Ptr namePool() const = 0;

        /**
         * @short Returns the item that @c fn:current() returns.
         *
         * Hence, this is not the focus, and very different from the focus.
         *
         * @see CurrentItemStore
         * @see CurrentFN
         */
        virtual Item currentItem() const = 0;

        DynamicContext::Ptr createFocus();
        DynamicContext::Ptr createStack();
        DynamicContext::Ptr createReceiverContext(QAbstractXmlReceiver *const receiver);

        /**
         * Whenever a tree gets built, this function is called. DynamicContext
         * has the responsibility of keeping a copy of @p nm, such that it
         * doesn't go out of scope, since no one else will reference @p nm.
         *
         * I think this is currently only used for temporary node trees. In
         * other cases they are stored in the ExternalResourceLoader.
         *
         * The caller guarantees that @p nm is not @c null.
         */
        virtual void addNodeModel(const QAbstractXmlNodeModel::Ptr &nm) = 0;

        /**
         * Same as itemCacheCell(), but is only used for global varibles. This
         * is needed because sometimes stack frames needs to be created for
         * other kinds of variables(such as in the case of user function
         * calls), while the global variable(s) needs to continue to use the
         * same cache, instead of one for each new stack frame, typically an
         * instance of StackContextBase.
         *
         * This has two effects:
         *
         * - It's an optimization. Instead of that a global variable gets evaluated each
         * time a user function is called, think recursive functions, it's done
         * only once.
         * - Query stability, hence affects things like node identity and
         * therefore conformance. Hence affects for instance what nodes a query
         * returns, since node identity affect node deduplication.
         */
        virtual ItemCacheCell &globalItemCacheCell(const VariableSlotID slot) = 0;

        /**
         * @short When a template is called, this member carries the template
         * parameters.
         *
         * Hence this is similar to the other variable stack functions such as
         * rangeVariable() and expressionVariable(), the difference being that
         * the order of template parameters as well as its arguments can appear
         * in arbitrary order. Hence the name is used to make the order
         * insignificant.
         */
        virtual TemplateParameterHash &templateParameterStore() = 0;

        /**
         * Same as itemSequenceCacheCells() but applies only for global
         * variables.
         *
         * @see globalItemCacheCell()
         */
        virtual ItemSequenceCacheCell::Vector &globalItemSequenceCacheCells(const VariableSlotID slot) = 0;

        /**
         * @short Returns the previous DynamicContext. If this context is the
         * top-level one, @c null is returned.
         */
        virtual DynamicContext::Ptr previousContext() const = 0;

        /**
         * @short Returns the current template mode that is in effect.
         *
         * If @c null is returned, it means that the default mode should be
         * used as the current mode.
         */
        virtual QExplicitlySharedDataPointer<TemplateMode> currentTemplateMode() const = 0;
    };
}

QT_END_NAMESPACE

#endif
