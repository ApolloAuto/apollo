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

#ifndef ULTRAPARTICLE_H
#define ULTRAPARTICLE_H
#include "qquickparticlepainter_p.h"
#include "qquickdirection_p.h"
#include <private/qquickpixmapcache_p.h>
#include <QQmlListProperty>
#include <QtQuick/qsgsimplematerial.h>
#include <QtGui/qcolor.h>

QT_BEGIN_NAMESPACE

class ImageMaterialData;
class QSGGeometryNode;

class QQuickSprite;
class QQuickStochasticEngine;

struct SimpleVertex {
    float x;
    float y;
    float t;
    float lifeSpan;
    float size;
    float endSize;
    float vx;
    float vy;
    float ax;
    float ay;
};

struct ColoredVertex {
    float x;
    float y;
    float t;
    float lifeSpan;
    float size;
    float endSize;
    float vx;
    float vy;
    float ax;
    float ay;
    Color4ub color;
};

struct DeformableVertex {
    float x;
    float y;
    float tx;
    float ty;
    float t;
    float lifeSpan;
    float size;
    float endSize;
    float vx;
    float vy;
    float ax;
    float ay;
    Color4ub color;
    float xx;
    float xy;
    float yx;
    float yy;
    float rotation;
    float rotationVelocity;
    float autoRotate;//Assumed that GPUs prefer floats to bools
};

struct SpriteVertex {
    float x;
    float y;
    float tx;
    float ty;
    float t;
    float lifeSpan;
    float size;
    float endSize;
    float vx;
    float vy;
    float ax;
    float ay;
    Color4ub color;
    float xx;
    float xy;
    float yx;
    float yy;
    float rotation;
    float rotationVelocity;
    float autoRotate;//Assumed that GPUs prefer floats to bools
    float animW;
    float animH;
    float animProgress;
    float animX1;
    float animY1;
    float animX2;
    float animY2;
};

template <typename Vertex>
struct Vertices {
    Vertex v1;
    Vertex v2;
    Vertex v3;
    Vertex v4;
};

class QQuickImageParticle : public QQuickParticlePainter
{
    Q_OBJECT
    Q_PROPERTY(QUrl source READ image WRITE setImage NOTIFY imageChanged)
    Q_PROPERTY(QQmlListProperty<QQuickSprite> sprites READ sprites)
    Q_PROPERTY(Status status READ status NOTIFY statusChanged)
    //### Is it worth having progress like Image has?
    //Q_PROPERTY(qreal progress READ progress NOTIFY progressChanged)

    Q_PROPERTY(QUrl colorTable READ colortable WRITE setColortable NOTIFY colortableChanged)
    Q_PROPERTY(QUrl sizeTable READ sizetable WRITE setSizetable NOTIFY sizetableChanged)
    Q_PROPERTY(QUrl opacityTable READ opacitytable WRITE setOpacitytable NOTIFY opacitytableChanged)

    //###Now just colorize - add a flag for 'solid' color particles(where the img is just a mask?)?
    Q_PROPERTY(QColor color READ color WRITE setColor NOTIFY colorChanged RESET resetColor)
    //Stacks (added) with individual colorVariations
    Q_PROPERTY(qreal colorVariation READ colorVariation WRITE setColorVariation NOTIFY colorVariationChanged RESET resetColor)
    Q_PROPERTY(qreal redVariation READ redVariation WRITE setRedVariation NOTIFY redVariationChanged RESET resetColor)
    Q_PROPERTY(qreal greenVariation READ greenVariation WRITE setGreenVariation NOTIFY greenVariationChanged RESET resetColor)
    Q_PROPERTY(qreal blueVariation READ blueVariation WRITE setBlueVariation NOTIFY blueVariationChanged RESET resetColor)
    //Stacks (multiplies) with the Alpha in the color, mostly here so you can use svg color names (which have full alpha)
    Q_PROPERTY(qreal alpha READ alpha WRITE setAlpha NOTIFY alphaChanged RESET resetColor)
    Q_PROPERTY(qreal alphaVariation READ alphaVariation WRITE setAlphaVariation NOTIFY alphaVariationChanged RESET resetColor)

    Q_PROPERTY(qreal rotation READ rotation WRITE setRotation NOTIFY rotationChanged RESET resetRotation)
    Q_PROPERTY(qreal rotationVariation READ rotationVariation WRITE setRotationVariation NOTIFY rotationVariationChanged RESET resetRotation)
    Q_PROPERTY(qreal rotationVelocity READ rotationVelocity WRITE setRotationVelocity NOTIFY rotationVelocityChanged RESET resetRotation)
    Q_PROPERTY(qreal rotationVelocityVariation READ rotationVelocityVariation WRITE setRotationVelocityVariation NOTIFY rotationVelocityVariationChanged RESET resetRotation)
    //If true, then will face the direction of motion. Stacks with rotation, e.g. setting rotation
    //to 180 will lead to facing away from the direction of motion
    Q_PROPERTY(bool autoRotation READ autoRotation WRITE setAutoRotation NOTIFY autoRotationChanged RESET resetRotation)

    //xVector is the vector from the top-left point to the top-right point, and is multiplied by current size
    Q_PROPERTY(QQuickDirection* xVector READ xVector WRITE setXVector NOTIFY xVectorChanged RESET resetDeformation)
    //yVector is the same, but top-left to bottom-left. The particle is always a parallelogram.
    Q_PROPERTY(QQuickDirection* yVector READ yVector WRITE setYVector NOTIFY yVectorChanged RESET resetDeformation)
    Q_PROPERTY(bool spritesInterpolate READ spritesInterpolate WRITE setSpritesInterpolate NOTIFY spritesInterpolateChanged)

    Q_PROPERTY(EntryEffect entryEffect READ entryEffect WRITE setEntryEffect NOTIFY entryEffectChanged)
    Q_ENUMS(EntryEffect)
    Q_ENUMS(Status)
public:
    explicit QQuickImageParticle(QQuickItem *parent = 0);
    virtual ~QQuickImageParticle();

    enum Status { Null, Ready, Loading, Error };

    QQmlListProperty<QQuickSprite> sprites();
    QQuickStochasticEngine* spriteEngine() {return m_spriteEngine;}

    enum EntryEffect {
        None = 0,
        Fade = 1,
        Scale = 2
    };

    enum PerformanceLevel{//TODO: Expose?
        Unknown = 0,
        Simple,
        Colored,
        Deformable,
        Tabled,
        Sprites
    };

    QUrl image() const { return m_image ? m_image->source : QUrl(); }
    void setImage(const QUrl &image);

    QUrl colortable() const { return m_colorTable ? m_colorTable->source : QUrl(); }
    void setColortable(const QUrl &table);

    QUrl sizetable() const { return m_sizeTable ? m_sizeTable->source : QUrl(); }
    void setSizetable (const QUrl &table);

    QUrl opacitytable() const { return m_opacityTable ? m_opacityTable->source : QUrl(); }
    void setOpacitytable(const QUrl &table);

    QColor color() const { return m_color; }
    void setColor(const QColor &color);

    qreal colorVariation() const { return m_color_variation; }
    void setColorVariation(qreal var);

    qreal alphaVariation() const { return m_alphaVariation; }

    qreal alpha() const { return m_alpha; }

    qreal redVariation() const { return m_redVariation; }

    qreal greenVariation() const { return m_greenVariation; }

    qreal blueVariation() const { return m_blueVariation; }

    qreal rotation() const { return m_rotation; }

    qreal rotationVariation() const { return m_rotationVariation; }

    qreal rotationVelocity() const { return m_rotationVelocity; }

    qreal rotationVelocityVariation() const { return m_rotationVelocityVariation; }

    bool autoRotation() const { return m_autoRotation; }

    QQuickDirection* xVector() const { return m_xVector; }

    QQuickDirection* yVector() const { return m_yVector; }

    bool spritesInterpolate() const { return m_spritesInterpolate; }

    bool bypassOptimizations() const { return m_bypassOptimizations; }

    EntryEffect entryEffect() const { return m_entryEffect; }

    Status status() const { return m_status; }

    void resetColor();
    void resetRotation();
    void resetDeformation();

Q_SIGNALS:

    void imageChanged();
    void colortableChanged();
    void sizetableChanged();
    void opacitytableChanged();

    void colorChanged();
    void colorVariationChanged();

    void alphaVariationChanged(qreal arg);

    void alphaChanged(qreal arg);

    void redVariationChanged(qreal arg);

    void greenVariationChanged(qreal arg);

    void blueVariationChanged(qreal arg);

    void rotationChanged(qreal arg);

    void rotationVariationChanged(qreal arg);

    void rotationVelocityChanged(qreal arg);

    void rotationVelocityVariationChanged(qreal arg);

    void autoRotationChanged(bool arg);

    void xVectorChanged(QQuickDirection* arg);

    void yVectorChanged(QQuickDirection* arg);

    void spritesInterpolateChanged(bool arg);

    void bypassOptimizationsChanged(bool arg);

    void entryEffectChanged(EntryEffect arg);

    void statusChanged(Status arg);

public Q_SLOTS:
    void reloadColor(const Color4ub &c, QQuickParticleData* d);
    void setAlphaVariation(qreal arg);

    void setAlpha(qreal arg);

    void setRedVariation(qreal arg);

    void setGreenVariation(qreal arg);

    void setBlueVariation(qreal arg);

    void setRotation(qreal arg);

    void setRotationVariation(qreal arg);

    void setRotationVelocity(qreal arg);

    void setRotationVelocityVariation(qreal arg);

    void setAutoRotation(bool arg);

    void setXVector(QQuickDirection* arg);

    void setYVector(QQuickDirection* arg);

    void setSpritesInterpolate(bool arg);

    void setBypassOptimizations(bool arg);

    void setEntryEffect(EntryEffect arg);

protected:
    void reset();
    virtual void initialize(int gIdx, int pIdx);
    virtual void commit(int gIdx, int pIdx);

    QSGNode *updatePaintNode(QSGNode *, UpdatePaintNodeData *);
    void prepareNextFrame(QSGNode**);
    void buildParticleNodes(QSGNode**);

    void sceneGraphInvalidated();

private Q_SLOTS:
    void createEngine(); //### method invoked by sprite list changing (in engine.h) - pretty nasty

    void spriteAdvance(int spriteIndex);
    void spritesUpdate(qreal time = 0 );
    void mainThreadFetchImageData();
    void finishBuildParticleNodes(QSGNode **n);
private:
    struct ImageData {
        QUrl source;
        QQuickPixmap pix;
    };
    QScopedPointer<ImageData> m_image;
    QScopedPointer<ImageData> m_colorTable;
    QScopedPointer<ImageData> m_sizeTable;
    QScopedPointer<ImageData> m_opacityTable;
    bool loadingSomething();


    QColor m_color;
    qreal m_color_variation;

    QHash<int, QSGGeometryNode *> m_nodes;
    QHash<int, int> m_idxStarts;//TODO: Proper resizing will lead to needing a spriteEngine per particle - do this after sprite engine gains transparent sharing?
    QList<QPair<int, int> > m_startsIdx;//Same data, optimized for alternate retrieval

    int m_lastIdxStart;
    QSGMaterial *m_material;

    // derived values...

    qreal m_alphaVariation;
    qreal m_alpha;
    qreal m_redVariation;
    qreal m_greenVariation;
    qreal m_blueVariation;
    qreal m_rotation;
    qreal m_rotationVariation;
    qreal m_rotationVelocity;
    qreal m_rotationVelocityVariation;
    bool m_autoRotation;
    QQuickDirection* m_xVector;
    QQuickDirection* m_yVector;

    QList<QQuickSprite*> m_sprites;
    QQuickSpriteEngine* m_spriteEngine;
    bool m_spritesInterpolate;

    bool m_explicitColor;
    bool m_explicitRotation;
    bool m_explicitDeformation;
    bool m_explicitAnimation;
    QHash<int, QVector<QQuickParticleData*> > m_shadowData;
    void clearShadows();
    QQuickParticleData* getShadowDatum(QQuickParticleData* datum);

    bool m_bypassOptimizations;
    PerformanceLevel perfLevel;

    PerformanceLevel m_lastLevel;
    bool m_debugMode;

    template<class Vertex>
    void initTexCoords(Vertex* v, int count){
        Vertex* end = v + count;
        while (v < end){
            v[0].tx = 0;
            v[0].ty = 0;

            v[1].tx = 1;
            v[1].ty = 0;

            v[2].tx = 0;
            v[2].ty = 1;

            v[3].tx = 1;
            v[3].ty = 1;

            v += 4;
        }
    }

    template<class MaterialData>
    MaterialData* getState(QSGMaterial* m){
        return static_cast<QSGSimpleMaterial<MaterialData> *>(m)->state();
    }
    EntryEffect m_entryEffect;
    Status m_status;
    int m_startedImageLoading;
};

QT_END_NAMESPACE
#endif // ULTRAPARTICLE_H
