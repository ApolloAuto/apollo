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

#ifndef Patternist_SubsequenceIterator_H
#define Patternist_SubsequenceIterator_H

#include <private/qitem_p.h>

QT_BEGIN_NAMESPACE

namespace QPatternist
{
    /**
     * @short Picks out a slice from another QAbstractXmlForwardIterator, specified by a start and end position.
     *
     * SubsequenceIterator allows a "slice", a subsequence, from an QAbstractXmlForwardIterator to
     * be extracted. The SubsequenceIterator's constructor takes a source QAbstractXmlForwardIterator,
     * a start position, and the length of the subsequence to be extracted.
     *
     * SubsequenceIterator contains the central business logic to implement
     * the <tt>fn:subsequence()</tt> function. The detailed behavior, such as how it behaves
     * if the source QAbstractXmlForwardIterator is empty or if the specified subsequence stretches
     * beyond the source QAbstractXmlForwardIterator, is therefore consistent with the definition of
     * the <tt>fn:subsequence()</tt> function.
     *
     * @see <a href="http://www.w3.org/TR/xpath-functions/#func-subsequence">XQuery 1.0
     * and XPath 2.0 Functions and Operators, 15.1.10 fn:subsequence</a>
     * @author Frans Englich <frans.englich@nokia.com>
     * @ingroup Patternist_iterators
     */
    class SubsequenceIterator : public Item::Iterator
    {
    public:
        /**
         * Creates a SubsequenceIterator that extracts a subsequence from the sequence
         * in @p iterator, as specified by the @p start position and @p length parameter.
         *
         * @param iterator the iterator which the subsequence should
         * be extracted from
         * @param start the start position of extraction. Must be 1 or larger.
         * @param length the length of the subsequence to extract. If it is
         * -1, to the end is returned. The value must be -1 or 1 or larger.
         */
        SubsequenceIterator(const Item::Iterator::Ptr &iterator,
                            const xsInteger start,
                            const xsInteger length);

        virtual Item next();
        virtual Item current() const;
        virtual xsInteger position() const;
        virtual Item::Iterator::Ptr copy() const;

    private:
        xsInteger m_position;
        Item m_current;
        const Item::Iterator::Ptr m_it;
        xsInteger m_counter;
        const xsInteger m_start;
        const xsInteger m_len;
        const xsInteger m_stop;
    };
}

QT_END_NAMESPACE

#endif
