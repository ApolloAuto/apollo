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

#ifndef Patternist_SingletonIterator_H
#define Patternist_SingletonIterator_H

#include <QtXmlPatterns/private/qabstractxmlforwarditerator_p.h>

#include <QtXmlPatterns/private/qprimitives_p.h>

QT_BEGIN_NAMESPACE

namespace QPatternist
{
    /**
     * @short An QAbstractXmlForwardIterator over exactly one item.
     *
     * SingletonIterator's constructor takes one value which is
     * the item it forms an QAbstractXmlForwardIterator over. Other QAbstractXmlForwardIterator instances can
     * also form an QAbstractXmlForwardIterator with one in length, but by that SingletonIterator
     * has this as it only task, it means it is efficient at it.
     *
     * Having to represent single items in Iterators is relatively common.
     *
     * @author Frans Englich <frans.englich@nokia.com>
     * @ingroup Patternist_iterators
     */
    template<typename T>
    class SingletonIterator : public QAbstractXmlForwardIterator<T>
    {
    public:
        /**
         * Creates an iterator over @p item.
         *
         * @note item may not be @c null. Use the EmptyIterator for
         * the empty sequence
         */
        SingletonIterator(const T &item) : m_item(item),
                                           m_position(0)
        {
            Q_ASSERT(!qIsForwardIteratorEnd(item));
        }

        virtual T next()
        {
            switch(m_position)
            {
                case 0:
                {
                    ++m_position;
                    return m_item;
                }
                case 1:
                {
                    m_position = -1;
                    return T();
                }
                default:
                {
                    Q_ASSERT(m_position == -1);
                    return T();
                }
            }
        }

        virtual T current() const
        {
            if(m_position == 1)
                return m_item;
            else
                return T();
        }

        virtual xsInteger position() const
        {
            return m_position;
        }

        /**
         * @returns a copy of this instance, rewinded to the beginning.
         */
        virtual typename QAbstractXmlForwardIterator<T>::Ptr toReversed()
        {
            return typename QAbstractXmlForwardIterator<T>::Ptr(new SingletonIterator<T>(m_item));
        }

        /**
         * @returns always 1
         */
        virtual xsInteger count()
        {
            return 1;
        }

        virtual typename QAbstractXmlForwardIterator<T>::Ptr copy() const
        {
            return typename QAbstractXmlForwardIterator<T>::Ptr(new SingletonIterator(m_item));
        }

    private:
        const T m_item;
        qint8 m_position;
    };

    /**
     * @short An object generator for SingletonIterator.
     *
     * makeSingletonIterator() is a convenience function for avoiding specifying
     * the full template instantiation for SingletonIterator. Conceptually, it
     * is identical to Qt's qMakePair().
     *
     * @relates SingletonIterator
     */
    template<typename T>
    inline
    typename SingletonIterator<T>::Ptr
    makeSingletonIterator(const T &item)
    {
        return typename SingletonIterator<T>::Ptr(new SingletonIterator<T>(item));
    }
}

QT_END_NAMESPACE

#endif
