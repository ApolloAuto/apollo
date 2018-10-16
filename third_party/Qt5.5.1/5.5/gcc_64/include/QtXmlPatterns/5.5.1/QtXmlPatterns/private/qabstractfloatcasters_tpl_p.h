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
//

/*
 * NOTE: This file is included by qabstractfloatcasters_p.h
 * if you need some includes, put them in qabstractfloatcasters_p.h (outside of the namespace)
 */

template <const bool isDouble>
Item NumericToAbstractFloatCaster<isDouble>::castFrom(const Item &from,
                                                           const QExplicitlySharedDataPointer<DynamicContext> &) const
{
    // toDouble() returns same thing than toFloat()
    return toItem(AbstractFloat<isDouble>::fromValue(from.template as<Numeric>()->toDouble()));
}

template <const bool isDouble>
Item StringToAbstractFloatCaster<isDouble>::castFrom(const Item &from,
                                                          const QExplicitlySharedDataPointer<DynamicContext> &) const
{
    return toItem(AbstractFloat<isDouble>::fromLexical(from.stringValue()));
}

template <const bool isDouble>
Item BooleanToAbstractFloatCaster<isDouble>::castFrom(const Item &from,
                                                           const QExplicitlySharedDataPointer<DynamicContext> &context) const
{
    // RVCT doesn't like using template parameter in trinary operator when the trinary operator result is
    // passed directly into another constructor.
    bool tempDouble = isDouble;
    if(from.template as<AtomicValue>()->evaluateEBV(context))
        return tempDouble ? toItem(CommonValues::DoubleOne) : toItem(CommonValues::FloatOne);
    else
        return tempDouble ? toItem(CommonValues::DoubleZero) : toItem(CommonValues::FloatZero);
}

