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

#ifndef Patternist_SequenceMappingIterator_H
#define Patternist_SequenceMappingIterator_H

#include <private/qabstractxmlforwarditerator_p.h>
#include <private/qdynamiccontext_p.h>

QT_BEGIN_NAMESPACE

namespace QPatternist
{
    /**
     * @short Proxies another QAbstractXmlForwardIterator, and for each item, returns the
     * Sequence returned from a mapping function.
     *
     * ItemMappingIterator is practical when the items in an QAbstractXmlForwardIterator needs to
     * be translated to another sequence, while still doing it in a pipe-lined
     * fashion. In contrast to ItemMappingIterator, SequenceMappingIterator maps
     * each item into another QAbstractXmlForwardIterator, and where the SequenceMappingIterator's own
     * result is the concatenation of all those Iterators. Hence, while ItemMappingIterator
     * is better tailored for one-to-one or one-to-zero conversion, SequenceMappingIterator
     * is more suitable for one-to-many conversion.
     *
     * This is achieved by that SequenceMappingIterator's constructor takes
     * an instance of a class, that must have the following member:
     *
     * @code
     * QAbstractXmlForwardIterator<TResult>::Ptr mapToSequence(const TSource::Ptr &item,
     *                                                      const DynamicContext::Ptr &context) const;
     * @endcode
     *
     * @author Frans Englich <frans.englich@nokia.com>
     * @see ItemMappingIterator
     * @ingroup Patternist_iterators
     */
    template<typename TResult, typename TSource, typename TMapper>
    class SequenceMappingIterator : public QAbstractXmlForwardIterator<TResult>
    {
    public:
        /**
         * Constructs a SequenceMappingIterator.
         *
         * @param mapper the object that has the mapToItem() sequence.
         * @param sourceIterator the QAbstractXmlForwardIterator whose items should be mapped.
         * @param context the DynamicContext that will be passed to the map function.
         * May be null.
         */
        SequenceMappingIterator(const TMapper &mapper,
                                const typename QAbstractXmlForwardIterator<TSource>::Ptr &sourceIterator,
                                const DynamicContext::Ptr &context);

        virtual TResult next();
        virtual xsInteger count();
        virtual TResult current() const;
        virtual xsInteger position() const;

        /**
         * The reason the implementation is placed in line here, is due to a bug
         * in MSVC-2005 version 14.00.50727.762. Note that it works with version 14.00.50727.42.
         */
        virtual typename QAbstractXmlForwardIterator<TResult>::Ptr copy() const
        {
            return typename QAbstractXmlForwardIterator<TResult>::Ptr
                    (new SequenceMappingIterator<TResult, TSource, TMapper>(m_mapper,
                                                                            m_mainIterator->copy(),
                                                                            m_context));
        }

    private:
        xsInteger                                           m_position;
        TResult                                             m_current;
        typename QAbstractXmlForwardIterator<TSource>::Ptr  m_mainIterator;
        typename QAbstractXmlForwardIterator<TResult>::Ptr  m_currentIterator;
        const typename DynamicContext::Ptr                  m_context;
        const TMapper                                       m_mapper;
    };

    template<typename TResult, typename TSource, typename TMapper>
    SequenceMappingIterator<TResult, TSource, TMapper>::SequenceMappingIterator(
                                        const TMapper &mapper,
                                        const typename QAbstractXmlForwardIterator<TSource>::Ptr &iterator,
                                        const DynamicContext::Ptr &context)
                                        : m_position(0),
                                          m_mainIterator(iterator),
                                          m_context(context),
                                          m_mapper(mapper)
    {
        Q_ASSERT(mapper);
        Q_ASSERT(iterator);
    }

    template<typename TResult, typename TSource, typename TMapper>
    TResult SequenceMappingIterator<TResult, TSource, TMapper>::next()
    {
        /* This was once implemented with a recursive function, but the stack
         * got blown for some inputs by that approach. */
        while(true)
        {
            while(!m_currentIterator)
            {
                const TSource mainItem(m_mainIterator->next());

                if(qIsForwardIteratorEnd(mainItem)) /* We've reached the very end. */
                {
                    m_position = -1;
                    m_current = TResult();
                    return TResult();
                }
                else
                    m_currentIterator = m_mapper->mapToSequence(mainItem, m_context);
            }

            m_current = m_currentIterator->next();

            if(qIsForwardIteratorEnd(m_current))
            {
                m_currentIterator.reset();
                continue;
            }
            else
            {
                ++m_position;
                return m_current;
            }
        }
    }

    template<typename TResult, typename TSource, typename TMapper>
    xsInteger SequenceMappingIterator<TResult, TSource, TMapper>::count()
    {
        TSource unit(m_mainIterator->next());
        xsInteger c = 0;

        while(!qIsForwardIteratorEnd(unit))
        {
            const typename QAbstractXmlForwardIterator<TResult>::Ptr sit(m_mapper->mapToSequence(unit, m_context));
            c += sit->count();
            unit = m_mainIterator->next();
        }

        return c;
    }

    template<typename TResult, typename TSource, typename TMapper>
    TResult SequenceMappingIterator<TResult, TSource, TMapper>::current() const
    {
        return m_current;
    }

    template<typename TResult, typename TSource, typename TMapper>
    xsInteger SequenceMappingIterator<TResult, TSource, TMapper>::position() const
    {
        return m_position;
    }


    /**
     * @short An object generator for SequenceMappingIterator.
     *
     * makeSequenceMappingIterator() is a convenience function for avoiding specifying
     * the full template instantiation for SequenceMappingIterator. Conceptually, it
     * is identical to Qt's qMakePair().
     *
     * @returns a SequenceMappingIterator wrapped in a smart pointer, that has been
     * passed the constructor arguments @p mapper, @p source, and @p context.
     * @see makeMappingCallbackPtr()
     * @relates QAbstractXmlForwardIterator
     */
    template<typename TResult, typename TSource, typename TMapper>
    static inline
    typename QAbstractXmlForwardIterator<TResult>::Ptr
    makeSequenceMappingIterator(const TMapper &mapper,
                                const QExplicitlySharedDataPointer<QAbstractXmlForwardIterator<TSource> > &source,
                                const DynamicContext::Ptr &context)
    {
        return typename QAbstractXmlForwardIterator<TResult>::Ptr
            (new SequenceMappingIterator<TResult, TSource, TMapper>(mapper, source, context));
    }
}

QT_END_NAMESPACE

#endif
