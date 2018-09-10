/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the XMLPatterns module of the Qt Toolkit.
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

#ifndef Patternist_ColoringMessageHandler_h
#define Patternist_ColoringMessageHandler_h

#include <QHash>

#include <private/qcoloroutput_p.h>
#include <QAbstractMessageHandler>

QT_BEGIN_NAMESPACE

namespace QPatternist
{
    class ColoringMessageHandler : public QAbstractMessageHandler
                                 , private ColorOutput
    {
    public:
        ColoringMessageHandler(QObject *parent = 0);

    protected:
        virtual void handleMessage(QtMsgType type,
                                   const QString &description,
                                   const QUrl &identifier,
                                   const QSourceLocation &sourceLocation);

    private:
        QString colorifyDescription(const QString &in) const;

        enum ColorType
        {
            RunningText,
            Location,
            ErrorCode,
            Keyword,
            Data
        };

        QHash<QString, ColorType> m_classToColor;
    };
}

QT_END_NAMESPACE

#endif
