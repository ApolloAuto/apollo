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

#ifndef Patternist_CompressedWhitespace_H
#define Patternist_CompressedWhitespace_H

#include <QtGlobal>

QT_BEGIN_NAMESPACE

class QChar;
class QString;
class QStringRef;

namespace QPatternist
{
    /**
     * @short A compression facility for whitespace nodes.
     *
     * CompressedWhitespace compresses and decompresses strings that consists of
     * whitespace only, and do so with a scheme that is designed to do this
     * specialized task in an efficient way. The approach is simple: each
     * sequence of equal whitespace in the input gets coded into one byte,
     * where the first two bits signals the type, CharIdentifier, and the
     * remininding six bits is the count.
     *
     * For instance, this scheme manages to compress a sequence of spaces
     * followed by a new line into 16 bits(one QChar), and QString stores
     * strings of one QChar quite efficiently, by avoiding a heap allocation.
     *
     * There is no way to tell whether a QString is compressed or not.
     *
     * The compression scheme originates from Saxon, by Michael Kay.
     *
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class Q_AUTOTEST_EXPORT CompressedWhitespace
    {
        public:
            /**
             * @short Compresses @p input into a compressed format, returned
             * as a QString.
             *
             * The caller guarantees that input is not empty
             * and consists only of whitespace.
             *
             * The returned format is opaque. There is no way to find out
             * whether a QString contains compressed data or not.
             *
             * @see decompress()
             */
            static QString compress(const QStringRef &input);

            /**
             * @short Decompresses @p input into a usual QString.
             *
             * @p input must be a QString as per returned from compress().
             *
             * @see compress()
             */
            static QString decompress(const QString &input);

        private:
            /**
             * We use the two upper bits for communicating what space it is.
             */
            enum CharIdentifier
            {
                Space   = 0x0,

                /**
                 * 0xA, \\r
                 *
                 * Binary: 10000000
                 */
                CR      = 0x80,

                /**
                 * 0xD, \\n
                 *
                 * Binary: 01000000
                 */
                LF      = 0x40,

                /**
                 * Binary: 11000000
                 */
                Tab     = 0xC0
            };

            enum Constants
            {
                /* We can at maximum store this many consecutive characters
                 * of one type. We use 6 bits for the count. */
                MaxCharCount = (1 << 6) - 1,

                /**
                 * Binary: 11111111
                 */
                Lower8Bits = (1 << 8) - 1,

                /**
                 * Binary: 111111
                 */
                Lower6Bits = (1 << 6) - 1,

                /*
                 * Binary: 11000000
                 */
                UpperTwoBits = 3 << 6
            };

            static inline CharIdentifier toIdentifier(const QChar ch);

            static inline quint8 toCompressedChar(const QChar ch, const int len);
            static inline QChar toChar(const CharIdentifier id);

            /**
             * @short Returns @c true if @p number is an even number, otherwise
             * @c false.
             */
            static inline bool isEven(const int number);

            /**
             * @short This class can only be used via its static members.
             */
            inline CompressedWhitespace();
            Q_DISABLE_COPY(CompressedWhitespace)
    };
}

QT_END_NAMESPACE

#endif
