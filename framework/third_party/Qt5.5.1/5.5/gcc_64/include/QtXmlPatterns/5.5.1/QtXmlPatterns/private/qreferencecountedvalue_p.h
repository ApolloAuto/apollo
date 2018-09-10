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

#ifndef QPatternist_ReferenceCountedValue_p_h
#define QPatternist_ReferenceCountedValue_p_h

QT_BEGIN_NAMESPACE

namespace QPatternist
{
/*!
   \class ReferenceCountedValue
   \internal
   \since 4.4
   \brief A template class that reference counts a value.

   This class is useful when an instance needs to have ownership semantics
   as if it was value based. A typical examples is a QObject pointer, which
   doesn't have a single owner.

   This is achieved through storing a copy of the object as
   a member inside ReferenceCountedValue, which never is copied. It will
   stay in scope until the last reference to the ReferenceCountedValue instance
   is removed, and subsequently ReferenceCountedValue is deleted and hence also
   the contained value. One should use ReferenceCountedValue by passing around
   copies of Ptr, which is a typedef for the QExplicitlySharedDataPointer
   smart pointer.
*/
    template<typename T>
    class ReferenceCountedValue : public QSharedData
    {
    public:
        typedef QExplicitlySharedDataPointer<ReferenceCountedValue<T> > Ptr;

        inline ReferenceCountedValue(T *const v) : value(v)
        {
        }

        inline ~ReferenceCountedValue()
        {
            delete value;
        }

        T *const value;
    private:
        /*!
          Disabled, no implementation provided.
         */
        inline ReferenceCountedValue();
        Q_DISABLE_COPY(ReferenceCountedValue)
    };
}

QT_END_NAMESPACE

#endif
