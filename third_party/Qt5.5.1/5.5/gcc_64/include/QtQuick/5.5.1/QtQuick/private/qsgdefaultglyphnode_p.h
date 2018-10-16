/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the QtQuick module of the Qt Toolkit.
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

#ifndef QSGDEFAULTGLYPHNODE_P_H
#define QSGDEFAULTGLYPHNODE_P_H

#include <private/qsgadaptationlayer_p.h>
#include <QtQuick/qsgnode.h>

QT_BEGIN_NAMESPACE

class QGlyphs;
class QSGTextMaskMaterial;
class QSGDefaultGlyphNode: public QSGGlyphNode
{
public:
    QSGDefaultGlyphNode();
    virtual ~QSGDefaultGlyphNode();

    virtual QPointF baseLine() const { return m_baseLine; }
    virtual void setGlyphs(const QPointF &position, const QGlyphRun &glyphs);
    virtual void setColor(const QColor &color);

    virtual void setPreferredAntialiasingMode(AntialiasingMode) { }
    virtual void setStyle(QQuickText::TextStyle);
    virtual void setStyleColor(const QColor &);

    virtual void update();

protected:
    QGlyphRun m_glyphs;
    QPointF m_position;
    QColor m_color;
    QQuickText::TextStyle m_style;
    QColor m_styleColor;

    QPointF m_baseLine;
    QSGTextMaskMaterial *m_material;

    QSGGeometry m_geometry;
};

QT_END_NAMESPACE

#endif
