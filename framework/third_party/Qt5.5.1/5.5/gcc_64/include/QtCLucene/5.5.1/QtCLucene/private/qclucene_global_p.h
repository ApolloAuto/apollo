/****************************************************************************
**
** Copyright (C) 2003-2006 Ben van Klinken and the CLucene Team.
** All rights reserved.
**
** Portion Copyright (C) 2015 The Qt Company Ltd.
** All rights reserved.
**
** This file may be used under the terms of the GNU Lesser General Public
** License version 2.1 as published by the Free Software Foundation and
** appearing in the file LICENSE.LGPL included in the packaging of this file.
** Please review the following information to ensure the GNU Lesser General
** Public License version 2.1 requirements will be met:
** http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html.
**
****************************************************************************/

#ifndef QCLUCENE_GLOBAL_P_H
#define QCLUCENE_GLOBAL_P_H

//
//  W A R N I N G
//  -------------
//
// This file is not part of the Qt API. It exists for the convenience
// of the help generator tools. This header file may change from version
// to version without notice, or even be removed.
//
// We mean it.
//

#if !defined(_MSC_VER)
#   include "qclucene-config_p.h"
#endif

#include <QtCore/QChar>
#include <QtCore/QString>

#if !defined(_MSC_VER) && !defined(__MINGW32__) && defined(_CL_HAVE_WCHAR_H) && defined(_CL_HAVE_WCHAR_T)
#   if !defined(TCHAR)
#       define TCHAR wchar_t
#   endif
#else
#   include <qt_windows.h>
#endif

QT_BEGIN_NAMESPACE

#ifdef QT_STATIC
#   define Q_CLUCENE_EXPORT
#elif defined(QT_BUILD_CLUCENE_LIB)
#   define Q_CLUCENE_EXPORT Q_DECL_EXPORT
#else
#   define Q_CLUCENE_EXPORT Q_DECL_IMPORT
#endif

//
//  W A R N I N G
//  -------------
//
// adjustments here, need to be done in
// QTDIR/src/3rdparty/clucene/src/CLucene/StdHeader.h as well
//
#if defined(_LUCENE_DONTIMPLEMENT_NS_MACROS)

#elif !defined(DISABLE_NAMESPACE)
#   ifdef QT_NAMESPACE
#       define CL_NS_DEF(sub) namespace QT_NAMESPACE { namespace lucene{ namespace sub{
#       define CL_NS_DEF2(sub,sub2) namespace QT_NAMESPACE { namespace lucene{ namespace sub{ namespace sub2 {

#       define CL_NS_END }}}
#       define CL_NS_END2 }}}}

#       define CL_NS_USE(sub) using namespace QT_NAMESPACE::lucene::sub;
#       define CL_NS_USE2(sub,sub2) using namespace QT_NAMESPACE::lucene::sub::sub2;

#       define CL_NS(sub) QT_NAMESPACE::lucene::sub
#       define CL_NS2(sub,sub2) QT_NAMESPACE::lucene::sub::sub2
#   else
#       define CL_NS_DEF(sub) namespace lucene{ namespace sub{
#       define CL_NS_DEF2(sub,sub2) namespace lucene{ namespace sub{ namespace sub2 {

#       define CL_NS_END }}
#       define CL_NS_END2 }}}

#       define CL_NS_USE(sub) using namespace lucene::sub;
#       define CL_NS_USE2(sub,sub2) using namespace lucene::sub::sub2;

#       define CL_NS(sub) lucene::sub
#       define CL_NS2(sub,sub2) lucene::sub::sub2
#   endif
#else
#   define CL_NS_DEF(sub)
#   define CL_NS_DEF2(sub, sub2)
#   define CL_NS_END
#   define CL_NS_END2
#   define CL_NS_USE(sub)
#   define CL_NS_USE2(sub,sub2)
#   define CL_NS(sub)
#   define CL_NS2(sub,sub2)
#endif

namespace QtCLuceneHelpers {
    inline TCHAR* QStringToTChar(const QString &str)
    {
        TCHAR *string = new TCHAR[(str.length() +1) * sizeof(TCHAR)];
        memset(string, 0, (str.length() +1) * sizeof(TCHAR));
        #if defined(UNICODE) || defined(_CL_HAVE_WCHAR_H) && defined(_CL_HAVE_WCHAR_T)
            str.toWCharArray(string);
        #else
            const QByteArray ba = str.toLatin1();
            strcpy(string, ba.constData());
        #endif
        return string;
    }

    inline QString TCharToQString(const TCHAR *string)
    {
        #if defined(UNICODE) || defined(_CL_HAVE_WCHAR_H) && defined(_CL_HAVE_WCHAR_T)
            QString retValue = QString::fromWCharArray(string);
            return retValue;
        #else
            return QString(QLatin1String(string));
        #endif
    }
}
using namespace QtCLuceneHelpers;

QT_END_NAMESPACE

#endif // QCLUCENE_GLOBAL_P_H
