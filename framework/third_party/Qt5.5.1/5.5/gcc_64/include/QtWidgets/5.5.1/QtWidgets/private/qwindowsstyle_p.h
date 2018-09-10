/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the QtWidgets module of the Qt Toolkit.
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

#ifndef QWINDOWSSTYLE_P_H
#define QWINDOWSSTYLE_P_H

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

#include <QtWidgets/qcommonstyle.h>

QT_BEGIN_NAMESPACE


#if !defined(QT_NO_STYLE_WINDOWS)

class QWindowsStylePrivate;

class QWindowsStyle : public QCommonStyle
{
    Q_OBJECT
public:
    QWindowsStyle();
    ~QWindowsStyle();

    void polish(QApplication*) Q_DECL_OVERRIDE;
    void unpolish(QApplication*) Q_DECL_OVERRIDE;

    void polish(QWidget*) Q_DECL_OVERRIDE;
    void unpolish(QWidget*) Q_DECL_OVERRIDE;

    void polish(QPalette &) Q_DECL_OVERRIDE;

    void drawPrimitive(PrimitiveElement pe, const QStyleOption *opt, QPainter *p,
                       const QWidget *w = 0) const Q_DECL_OVERRIDE;
    void drawControl(ControlElement element, const QStyleOption *opt, QPainter *p,
                     const QWidget *w = 0) const Q_DECL_OVERRIDE;
    QRect subElementRect(SubElement r, const QStyleOption *opt, const QWidget *widget = 0) const Q_DECL_OVERRIDE;
    void drawComplexControl(ComplexControl cc, const QStyleOptionComplex *opt, QPainter *p,
                            const QWidget *w = 0) const Q_DECL_OVERRIDE;
    QSize sizeFromContents(ContentsType ct, const QStyleOption *opt,
                           const QSize &contentsSize, const QWidget *widget = 0) const Q_DECL_OVERRIDE;

    int pixelMetric(PixelMetric pm, const QStyleOption *option = 0, const QWidget *widget = 0) const Q_DECL_OVERRIDE;

    int styleHint(StyleHint hint, const QStyleOption *opt = 0, const QWidget *widget = 0,
                  QStyleHintReturn *returnData = 0) const Q_DECL_OVERRIDE;

    QPixmap standardPixmap(StandardPixmap standardPixmap, const QStyleOption *opt,
                           const QWidget *widget = 0) const Q_DECL_OVERRIDE;

    QIcon standardIcon(StandardPixmap standardIcon, const QStyleOption *option = 0,
                       const QWidget *widget = 0) const Q_DECL_OVERRIDE;

protected:
    bool eventFilter(QObject *o, QEvent *e) Q_DECL_OVERRIDE;
    QWindowsStyle(QWindowsStylePrivate &dd);

private:
    Q_DISABLE_COPY(QWindowsStyle)
    Q_DECLARE_PRIVATE(QWindowsStyle)
    void *reserved;
};

#endif // QT_NO_STYLE_WINDOWS

QT_END_NAMESPACE

#endif // QWINDOWSSTYLE_P_H
