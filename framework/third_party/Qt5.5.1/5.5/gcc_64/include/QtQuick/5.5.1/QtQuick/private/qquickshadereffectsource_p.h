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

#ifndef QQUICKSHADEREFFECTSOURCE_P_H
#define QQUICKSHADEREFFECTSOURCE_P_H

#include "qquickitem.h"
#include <QtQuick/qsgtextureprovider.h>
#include <private/qsgadaptationlayer_p.h>
#include <QtQuick/private/qsgcontext_p.h>
#include <private/qsgdefaultimagenode_p.h>
#include <private/qquickitemchangelistener_p.h>

#include "qpointer.h"
#include "qsize.h"
#include "qrect.h"

QT_BEGIN_NAMESPACE

class QSGNode;
class UpdatePaintNodeData;
class QOpenGLFramebufferObject;
class QSGSimpleRectNode;

class QQuickShaderEffectSourceTextureProvider;

class Q_QUICK_PRIVATE_EXPORT QQuickShaderEffectSource : public QQuickItem, public QQuickItemChangeListener
{
    Q_OBJECT
    Q_PROPERTY(WrapMode wrapMode READ wrapMode WRITE setWrapMode NOTIFY wrapModeChanged)
    Q_PROPERTY(QQuickItem *sourceItem READ sourceItem WRITE setSourceItem NOTIFY sourceItemChanged)
    Q_PROPERTY(QRectF sourceRect READ sourceRect WRITE setSourceRect NOTIFY sourceRectChanged)
    Q_PROPERTY(QSize textureSize READ textureSize WRITE setTextureSize NOTIFY textureSizeChanged)
    Q_PROPERTY(Format format READ format WRITE setFormat NOTIFY formatChanged)
    Q_PROPERTY(bool live READ live WRITE setLive NOTIFY liveChanged)
    Q_PROPERTY(bool hideSource READ hideSource WRITE setHideSource NOTIFY hideSourceChanged)
    Q_PROPERTY(bool mipmap READ mipmap WRITE setMipmap NOTIFY mipmapChanged)
    Q_PROPERTY(bool recursive READ recursive WRITE setRecursive NOTIFY recursiveChanged)

    Q_ENUMS(Format WrapMode)
public:
    enum WrapMode {
        ClampToEdge,
        RepeatHorizontally,
        RepeatVertically,
        Repeat
    };

    enum Format {
        Alpha = GL_ALPHA,
        RGB = GL_RGB,
        RGBA = GL_RGBA
    };

    QQuickShaderEffectSource(QQuickItem *parent = 0);
    ~QQuickShaderEffectSource();

    WrapMode wrapMode() const;
    void setWrapMode(WrapMode mode);

    QQuickItem *sourceItem() const;
    void setSourceItem(QQuickItem *item);

    QRectF sourceRect() const;
    void setSourceRect(const QRectF &rect);

    QSize textureSize() const;
    void setTextureSize(const QSize &size);

    Format format() const;
    void setFormat(Format format);

    bool live() const;
    void setLive(bool live);

    bool hideSource() const;
    void setHideSource(bool hide);

    bool mipmap() const;
    void setMipmap(bool enabled);

    bool recursive() const;
    void setRecursive(bool enabled);

    bool isTextureProvider() const Q_DECL_OVERRIDE { return true; }
    QSGTextureProvider *textureProvider() const Q_DECL_OVERRIDE;

    Q_INVOKABLE void scheduleUpdate();

Q_SIGNALS:
    void wrapModeChanged();
    void sourceItemChanged();
    void sourceRectChanged();
    void textureSizeChanged();
    void formatChanged();
    void liveChanged();
    void hideSourceChanged();
    void mipmapChanged();
    void recursiveChanged();

    void scheduledUpdateCompleted();

private Q_SLOTS:
    void sourceItemDestroyed(QObject *item);
    void invalidateSceneGraph();

protected:
    void releaseResources() Q_DECL_OVERRIDE;
    QSGNode *updatePaintNode(QSGNode *, UpdatePaintNodeData *) Q_DECL_OVERRIDE;

    void itemGeometryChanged(QQuickItem *item, const QRectF &newRect, const QRectF &oldRect) Q_DECL_OVERRIDE;
    void itemChange(ItemChange change, const ItemChangeData &value) Q_DECL_OVERRIDE;

private:
    void ensureTexture();

    QQuickShaderEffectSourceTextureProvider *m_provider;
    QSGLayer *m_texture;
    WrapMode m_wrapMode;
    QQuickItem *m_sourceItem;
    QRectF m_sourceRect;
    QSize m_textureSize;
    Format m_format;
    uint m_live : 1;
    uint m_hideSource : 1;
    uint m_mipmap : 1;
    uint m_recursive : 1;
    uint m_grab : 1;
};

QT_END_NAMESPACE

#endif // QQUICKSHADEREFFECTSOURCE_P_H
