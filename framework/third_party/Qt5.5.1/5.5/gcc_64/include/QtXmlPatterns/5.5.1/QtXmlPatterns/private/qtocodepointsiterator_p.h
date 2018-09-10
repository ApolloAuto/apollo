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

#ifndef Patternist_ToCodepointsIterator_H
#define Patternist_ToCodepointsIterator_H

#include <private/qitem_p.h>

QT_BEGIN_NAMESPACE

namespace QPatternist
{
    /**
     * @short Represents a stream of Unicode codepoints, which are computed from a string.
     *
     * ToCodepointsIterator takes in its constructor a string, whose Unicode characters'
     * codepoints it forms an QAbstractXmlForwardIterator over.
     *
     * @see <a href="http://www.w3.org/TR/xpath-functions/#func-string-to-codepoints">XQuery 1.0
     * and XPath 2.0 Functions and Operators, 7.2.2 fn:string-to-codepoints</a>
     * @see StringToCodepointsFN
     * @author Frans Englich <frans.englich@nokia.com>
     * @ingroup Patternist_iterators
     */
    class ToCodepointsIterator : public Item::Iterator
    {
    public:
        /**
         * Constructs a ToCodepointsIterator.
         *
         * @param string the string to retrieve Unicode codepoints from. Can not be
         * empty.
         */
        ToCodepointsIterator(const QString &string);
        virtual Item next();
        virtual Item current() const;
        virtual xsInteger position() const;
        virtual xsInteger count();
        virtual Item::Iterator::Ptr copy() const;

    private:
        const QString m_string;
        const int m_len;
        Item m_current;
        xsInteger m_position;
    };
}

QT_END_NAMESPACE

#endif
