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

#ifndef QSGNODEUPDATER_P_H
#define QSGNODEUPDATER_P_H

#include <private/qtquickglobal_p.h>
#include <QtGui/private/qdatabuffer_p.h>

QT_BEGIN_NAMESPACE

class QSGNode;
class QSGTransformNode;
class QSGClipNode;
class QSGOpacityNode;
class QSGGeometryNode;
class QMatrix4x4;
class QSGRenderNode;

class Q_QUICK_PRIVATE_EXPORT QSGNodeUpdater
{
public:
    QSGNodeUpdater();
    virtual ~QSGNodeUpdater();

    virtual void updateStates(QSGNode *n);
    virtual bool isNodeBlocked(QSGNode *n, QSGNode *root) const;

protected:
    virtual void enterTransformNode(QSGTransformNode *);
    virtual void leaveTransformNode(QSGTransformNode *);
    void enterClipNode(QSGClipNode *c);
    void leaveClipNode(QSGClipNode *c);
    void enterOpacityNode(QSGOpacityNode *o);
    void leaveOpacityNode(QSGOpacityNode *o);
    void enterGeometryNode(QSGGeometryNode *);
    void leaveGeometryNode(QSGGeometryNode *);
    void enterRenderNode(QSGRenderNode *);
    void leaveRenderNode(QSGRenderNode *);

    void visitNode(QSGNode *n);
    void visitChildren(QSGNode *n);


    QDataBuffer<const QMatrix4x4 *> m_combined_matrix_stack;
    QDataBuffer<qreal> m_opacity_stack;
    const QSGClipNode *m_current_clip;

    int m_force_update;
};

QT_END_NAMESPACE

#endif
