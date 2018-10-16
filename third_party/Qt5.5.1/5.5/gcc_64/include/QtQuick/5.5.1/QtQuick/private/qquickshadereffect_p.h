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

#ifndef QQUICKSHADEREFFECT_P_H
#define QQUICKSHADEREFFECT_P_H

#include <QtQuick/qquickitem.h>

#include <QtQuick/qsgmaterial.h>
#include <private/qtquickglobal_p.h>
#include <private/qsgadaptationlayer_p.h>
#include <private/qquickshadereffectnode_p.h>
#include "qquickshadereffectmesh_p.h"

#include <QtCore/qpointer.h>

QT_BEGIN_NAMESPACE

const char *qtPositionAttributeName();
const char *qtTexCoordAttributeName();

class QSGContext;
class QSignalMapper;
class QQuickCustomMaterialShader;

// Common class for QQuickShaderEffect and QQuickCustomParticle.
struct Q_QUICK_PRIVATE_EXPORT QQuickShaderEffectCommon
{
    typedef QQuickShaderEffectMaterialKey Key;
    typedef QQuickShaderEffectMaterial::UniformData UniformData;

    ~QQuickShaderEffectCommon();
    void disconnectPropertySignals(QQuickItem *item, Key::ShaderType shaderType);
    void connectPropertySignals(QQuickItem *item, Key::ShaderType shaderType);
    void updateParseLog(bool ignoreAttributes);
    void lookThroughShaderCode(QQuickItem *item, Key::ShaderType shaderType, const QByteArray &code);
    void updateShader(QQuickItem *item, Key::ShaderType shaderType);
    void updateMaterial(QQuickShaderEffectNode *node, QQuickShaderEffectMaterial *material,
                        bool updateUniforms, bool updateUniformValues, bool updateTextureProviders);
    void updateWindow(QQuickWindow *window);

    // Called by slots in QQuickShaderEffect:
    void sourceDestroyed(QObject *object);
    void propertyChanged(QQuickItem *item, int mappedId, bool *textureProviderChanged);

    Key source;
    QVector<QByteArray> attributes;
    QVector<UniformData> uniformData[Key::ShaderTypeCount];
    QVector<QSignalMapper *> signalMappers[Key::ShaderTypeCount];
    QString parseLog;
};


class Q_QUICK_PRIVATE_EXPORT QQuickShaderEffect : public QQuickItem
{
    Q_OBJECT
    Q_PROPERTY(QByteArray fragmentShader READ fragmentShader WRITE setFragmentShader NOTIFY fragmentShaderChanged)
    Q_PROPERTY(QByteArray vertexShader READ vertexShader WRITE setVertexShader NOTIFY vertexShaderChanged)
    Q_PROPERTY(bool blending READ blending WRITE setBlending NOTIFY blendingChanged)
    Q_PROPERTY(QVariant mesh READ mesh WRITE setMesh NOTIFY meshChanged)
    Q_PROPERTY(CullMode cullMode READ cullMode WRITE setCullMode NOTIFY cullModeChanged)
    Q_PROPERTY(QString log READ log NOTIFY logChanged)
    Q_PROPERTY(Status status READ status NOTIFY statusChanged)
    Q_PROPERTY(bool supportsAtlasTextures READ supportsAtlasTextures WRITE setSupportsAtlasTextures NOTIFY supportsAtlasTexturesChanged REVISION 1)
    Q_ENUMS(CullMode)
    Q_ENUMS(Status)

public:
    enum CullMode
    {
        NoCulling = QQuickShaderEffectMaterial::NoCulling,
        BackFaceCulling = QQuickShaderEffectMaterial::BackFaceCulling,
        FrontFaceCulling = QQuickShaderEffectMaterial::FrontFaceCulling
    };

    enum Status
    {
        Compiled,
        Uncompiled,
        Error
    };

    QQuickShaderEffect(QQuickItem *parent = 0);
    ~QQuickShaderEffect();

    QByteArray fragmentShader() const { return m_common.source.sourceCode[Key::FragmentShader]; }
    void setFragmentShader(const QByteArray &code);

    QByteArray vertexShader() const { return m_common.source.sourceCode[Key::VertexShader]; }
    void setVertexShader(const QByteArray &code);

    bool blending() const { return m_blending; }
    void setBlending(bool enable);

    QVariant mesh() const;
    void setMesh(const QVariant &mesh);

    CullMode cullMode() const { return m_cullMode; }
    void setCullMode(CullMode face);

    QString log() const { return m_log; }
    Status status() const { return m_status; }

    bool supportsAtlasTextures() const { return m_supportsAtlasTextures; }
    void setSupportsAtlasTextures(bool supports);

    QString parseLog();

    bool event(QEvent *) Q_DECL_OVERRIDE;

Q_SIGNALS:
    void fragmentShaderChanged();
    void vertexShaderChanged();
    void blendingChanged();
    void meshChanged();
    void cullModeChanged();
    void logChanged();
    void statusChanged();
    void supportsAtlasTexturesChanged();

protected:
    void geometryChanged(const QRectF &newGeometry, const QRectF &oldGeometry) Q_DECL_OVERRIDE;
    QSGNode *updatePaintNode(QSGNode *, UpdatePaintNodeData *) Q_DECL_OVERRIDE;
    void componentComplete() Q_DECL_OVERRIDE;
    void itemChange(ItemChange change, const ItemChangeData &value) Q_DECL_OVERRIDE;

private Q_SLOTS:
    void updateGeometry();
    void updateLogAndStatus(const QString &log, int status);
    void sourceDestroyed(QObject *object);
    void propertyChanged(int mappedId);

private:
    friend class QQuickCustomMaterialShader;
    friend class QQuickShaderEffectNode;

    typedef QQuickShaderEffectMaterialKey Key;
    typedef QQuickShaderEffectMaterial::UniformData UniformData;

    QSize m_meshResolution;
    QQuickShaderEffectMesh *m_mesh;
    QQuickGridMesh m_defaultMesh;
    CullMode m_cullMode;
    QString m_log;
    Status m_status;

    QQuickShaderEffectCommon m_common;

    uint m_blending : 1;
    uint m_dirtyUniforms : 1;
    uint m_dirtyUniformValues : 1;
    uint m_dirtyTextureProviders : 1;
    uint m_dirtyProgram : 1;
    uint m_dirtyParseLog : 1;
    uint m_dirtyMesh : 1;
    uint m_dirtyGeometry : 1;
    uint m_customVertexShader : 1;
    uint m_supportsAtlasTextures : 1;
};

QT_END_NAMESPACE

#endif // QQUICKSHADEREFFECT_P_H
