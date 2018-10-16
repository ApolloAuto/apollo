/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the QtQml module of the Qt Toolkit.
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

#ifndef CUSTOM_PARTICLE_H
#define CUSTOM_PARTICLE_H
#include "qquickparticlepainter_p.h"
#include <private/qquickshadereffectnode_p.h>
#include <private/qquickshadereffect_p.h>
#include <QSignalMapper>

QT_BEGIN_NAMESPACE

class QSGNode;
struct PlainVertices;

class QQuickShaderEffectMaterialObject;

//Genealogy: Hybrid of UltraParticle and ShaderEffect
class QQuickCustomParticle : public QQuickParticlePainter
{
    Q_OBJECT
    Q_PROPERTY(QByteArray fragmentShader READ fragmentShader WRITE setFragmentShader NOTIFY fragmentShaderChanged)
    Q_PROPERTY(QByteArray vertexShader READ vertexShader WRITE setVertexShader NOTIFY vertexShaderChanged)

public:
    explicit QQuickCustomParticle(QQuickItem* parent=0);
    ~QQuickCustomParticle();

    QByteArray fragmentShader() const { return m_common.source.sourceCode[Key::FragmentShader]; }
    void setFragmentShader(const QByteArray &code);

    QByteArray vertexShader() const { return m_common.source.sourceCode[Key::VertexShader]; }
    void setVertexShader(const QByteArray &code);

Q_SIGNALS:
    void fragmentShaderChanged();
    void vertexShaderChanged();

protected:
    virtual void initialize(int gIdx, int pIdx);
    virtual void commit(int gIdx, int pIdx);

    QSGNode *updatePaintNode(QSGNode *, UpdatePaintNodeData *);
    QQuickShaderEffectNode *prepareNextFrame(QQuickShaderEffectNode *rootNode);
    void reset();
    void resize(int oldCount, int newCount);
    virtual void componentComplete();
    QQuickShaderEffectNode *buildCustomNodes();

    void sceneGraphInvalidated();
    void itemChange(ItemChange change, const ItemChangeData &value);

private Q_SLOTS:
    void sourceDestroyed(QObject *object);
    void propertyChanged(int mappedId);

private:
    typedef QQuickShaderEffectMaterialKey Key;
    typedef QQuickShaderEffectMaterial::UniformData UniformData;

    void buildData(QQuickShaderEffectNode *rootNode);
    void updateVertexShader();

    QQuickShaderEffectCommon m_common;

    QHash<int, QQuickShaderEffectNode*> m_nodes;
    qreal m_lastTime;

    uint m_dirtyUniforms : 1;
    uint m_dirtyUniformValues : 1;
    uint m_dirtyTextureProviders : 1;
    uint m_dirtyProgram : 1;
};

QT_END_NAMESPACE

#endif //HEADER_GUARD
