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

#ifndef QSGRENDERER_P_H
#define QSGRENDERER_P_H

#include "qsgabstractrenderer.h"
#include "qsgabstractrenderer_p.h"
#include "qsgnode.h"
#include "qsgmaterial.h"

#include <QtQuick/private/qsgcontext_p.h>

QT_BEGIN_NAMESPACE

class QSGBindable;
class QSGNodeUpdater;

Q_QUICK_PRIVATE_EXPORT bool qsg_test_and_clear_fatal_render_error();
Q_QUICK_PRIVATE_EXPORT void qsg_set_fatal_renderer_error();

class Q_QUICK_PRIVATE_EXPORT QSGRenderer : public QSGAbstractRenderer
{
public:

    QSGRenderer(QSGRenderContext *context);
    virtual ~QSGRenderer();

    // Accessed by QSGMaterialShader::RenderState.
    QMatrix4x4 currentProjectionMatrix() const { return m_current_projection_matrix; }
    QMatrix4x4 currentModelViewMatrix() const { return m_current_model_view_matrix; }
    QMatrix4x4 currentCombinedMatrix() const { return m_current_projection_matrix * m_current_model_view_matrix; }
    qreal currentOpacity() const { return m_current_opacity; }
    qreal determinant() const { return m_current_determinant; }

    void setDevicePixelRatio(qreal ratio) { m_device_pixel_ratio = ratio; }
    qreal devicePixelRatio() const { return m_device_pixel_ratio; }
    QSGRenderContext *context() const { return m_context; }

    bool isMirrored() const;
    void renderScene(const QSGBindable &bindable);
    virtual void renderScene(GLuint fboId = 0) Q_DECL_OVERRIDE;
    virtual void nodeChanged(QSGNode *node, QSGNode::DirtyState state) Q_DECL_OVERRIDE;

    QSGNodeUpdater *nodeUpdater() const;
    void setNodeUpdater(QSGNodeUpdater *updater);

    inline QSGMaterialShader::RenderState state(QSGMaterialShader::RenderState::DirtyStates dirty) const;

    virtual void setCustomRenderMode(const QByteArray &) { };

    void clearChangedFlag() { m_changed_emitted = false; }

protected:
    virtual void render() = 0;

    const QSGBindable *bindable() const { return m_bindable; }

    virtual void preprocess();

    void addNodesToPreprocess(QSGNode *node);
    void removeNodesToPreprocess(QSGNode *node);

    QMatrix4x4 m_current_projection_matrix;
    QMatrix4x4 m_current_model_view_matrix;
    qreal m_current_opacity;
    qreal m_current_determinant;
    qreal m_device_pixel_ratio;

    QSGRenderContext *m_context;

private:
    QSGNodeUpdater *m_node_updater;

    QSet<QSGNode *> m_nodes_to_preprocess;

    const QSGBindable *m_bindable;

    uint m_changed_emitted : 1;
    uint m_is_rendering : 1;
};

class Q_QUICK_PRIVATE_EXPORT QSGBindable
{
public:
    virtual ~QSGBindable() { }
    virtual void bind() const = 0;
    virtual void clear(QSGAbstractRenderer::ClearMode mode) const;
    virtual void reactivate() const;
};

class QSGBindableFboId : public QSGBindable
{
public:
    QSGBindableFboId(GLuint);
    virtual void bind() const;
private:
    GLuint m_id;
};



QSGMaterialShader::RenderState QSGRenderer::state(QSGMaterialShader::RenderState::DirtyStates dirty) const
{
    QSGMaterialShader::RenderState s;
    s.m_dirty = dirty;
    s.m_data = this;
    return s;
}


class Q_QUICK_PRIVATE_EXPORT QSGNodeDumper : public QSGNodeVisitor {

public:
    static void dump(QSGNode *n);

    QSGNodeDumper() : m_indent(0) {}
    void visitNode(QSGNode *n);
    void visitChildren(QSGNode *n);

private:
    int m_indent;
};



QT_END_NAMESPACE

#endif
