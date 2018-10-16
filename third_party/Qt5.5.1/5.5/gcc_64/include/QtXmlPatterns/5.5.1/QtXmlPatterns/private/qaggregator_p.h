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

#ifndef Patternist_Aggregator_H
#define Patternist_Aggregator_H

#include <private/qfunctioncall_p.h>

QT_BEGIN_NAMESPACE

namespace QPatternist
{

    /**
     * @short Carries a staticType() implementation appropriate
     * for functions which returns a singleton value derived from its first argument.
     *
     * One example of such a function is FloorFN, implementing <tt>fn:floor()</tt>,
     * which returns a single value of the same type as the first argument, or the empty
     * sequence if the first argument evaluates to the empty sequence.
     *
     * Aggregator is abstract, and exists for saving code. It is inherited
     * by classes which needs the staticType() implementation this class provides.
     *
     * @see Piper
     * @ingroup Patternist_functions
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class Aggregator : public FunctionCall
    {
    public:
        /**
         * @returns a static type where the ItemType is the same as this FunctionCall's first
         * argument, and the Cardinality is as return from Cardinality::toWithoutMany().
         */
        virtual SequenceType::Ptr staticType() const;
    };
}

QT_END_NAMESPACE

#endif
