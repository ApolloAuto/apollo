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

#ifndef QQUICKSHADEREFFECTNODE_P_H
#define QQUICKSHADEREFFECTNODE_P_H

#include <QtQuick/qsgnode.h>
#include <QtQuick/qsgmaterial.h>
#include <QtQuick/qsgtextureprovider.h>
#include <QtQuick/qquickitem.h>
#include <private/qtquickglobal_p.h>

#include <QtCore/qsharedpointer.h>
#include <QtCore/qpointer.h>

QT_BEGIN_NAMESPACE

struct QQuickShaderEffectMaterialKey {
    enum ShaderType
    {
        VertexShader,
        FragmentShader,
        ShaderTypeCount
    };

    QByteArray sourceCode[ShaderTypeCount];
    const char *className;

    bool operator == (const QQuickShaderEffectMaterialKey &other) const;
    bool operator != (const QQuickShaderEffectMaterialKey &other) const;
};

uint qHash(const QQuickShaderEffectMaterialKey &key);


class QQuickCustomMaterialShader;
class QQuickShaderEffectNode;
class Q_QUICK_PRIVATE_EXPORT QQuickShaderEffectMaterial : public QSGMaterial
{
public:
    struct UniformData
    {
        enum SpecialType { None, Sampler, SubRect, Opacity, Matrix };

        QByteArray name;
        QVariant value;
        SpecialType specialType;

        bool operator == (const UniformData &other) const;
    };

    enum CullMode
    {
        NoCulling,
        BackFaceCulling,
        FrontFaceCulling
    };

    explicit QQuickShaderEffectMaterial(QQuickShaderEffectNode *node = 0);
    QSGMaterialType *type() const Q_DECL_OVERRIDE;
    QSGMaterialShader *createShader() const Q_DECL_OVERRIDE;
    int compare(const QSGMaterial *other) const Q_DECL_OVERRIDE;

    QVector<QByteArray> attributes;
    QVector<UniformData> uniforms[QQuickShaderEffectMaterialKey::ShaderTypeCount];
    QVector<QSGTextureProvider *> textureProviders;
    CullMode cullMode;
    bool geometryUsesTextureSubRect;

    void setProgramSource(const QQuickShaderEffectMaterialKey &source);
    void updateTextures() const;
    void invalidateTextureProvider(QSGTextureProvider *provider);

    static void cleanupMaterialCache();

protected:
    friend class QQuickCustomMaterialShader;

    // The type pointer needs to be unique. It is not safe to let the type object be part of the
    // QQuickShaderEffectMaterial, since it can be deleted and a new one constructed on top of the old
    // one. The new QQuickShaderEffectMaterial would then get the same type pointer as the old one, and
    // CustomMaterialShaders based on the old one would incorrectly be used together with the new
    // one. To guarantee that the type pointer is unique, the type object must live as long as
    // there are any CustomMaterialShaders of that type.
    QSharedPointer<QSGMaterialType> m_type;
    QQuickShaderEffectMaterialKey m_source;

    QQuickShaderEffectNode *m_node;
    bool m_emittedLogChanged;
};


class QSGShaderEffectMesh;

class Q_QUICK_PRIVATE_EXPORT QQuickShaderEffectNode : public QObject, public QSGGeometryNode
{
    Q_OBJECT
public:
    QQuickShaderEffectNode();
    virtual ~QQuickShaderEffectNode();

    void preprocess() Q_DECL_OVERRIDE;

Q_SIGNALS:
    void logAndStatusChanged(const QString &, int status);

private Q_SLOTS:
    void markDirtyTexture();
    void textureProviderDestroyed(QObject *object);
};

QT_END_NAMESPACE

#endif // QQUICKSHADEREFFECTNODE_P_H
