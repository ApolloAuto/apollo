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

#ifndef QSGABSTRACTRENDERER_P_H
#define QSGABSTRACTRENDERER_P_H

#include "qsgabstractrenderer.h"

#include "qsgnode.h"
#include <qcolor.h>

#include <QtCore/private/qobject_p.h>
#include <QtQuick/private/qtquickglobal_p.h>

QT_BEGIN_NAMESPACE

class Q_QUICK_PRIVATE_EXPORT QSGAbstractRendererPrivate : public QObjectPrivate
{
    Q_DECLARE_PUBLIC(QSGAbstractRenderer)
public:
    static const QSGAbstractRendererPrivate *get(const QSGAbstractRenderer *q) { return q->d_func(); }

    QSGAbstractRendererPrivate();
    void updateProjectionMatrix();

    QSGRootNode *m_root_node;
    QColor m_clear_color;
    QSGAbstractRenderer::ClearMode m_clear_mode;

    QRect m_device_rect;
    QRect m_viewport_rect;

    QMatrix4x4 m_projection_matrix;
    uint m_mirrored : 1;
};

QT_END_NAMESPACE

#endif
