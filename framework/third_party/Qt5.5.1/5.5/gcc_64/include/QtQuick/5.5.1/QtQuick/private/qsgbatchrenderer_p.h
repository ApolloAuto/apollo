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

#ifndef QSGBATCHRENDERER_P_H
#define QSGBATCHRENDERER_P_H

#include <private/qsgrenderer_p.h>
#include <private/qsgnodeupdater_p.h>
#include <private/qdatabuffer_p.h>

#include <private/qsgrendernode_p.h>

#include <QtCore/QBitArray>

QT_BEGIN_NAMESPACE

class QOpenGLVertexArrayObject;

namespace QSGBatchRenderer
{

#define QSG_RENDERER_COORD_LIMIT 1000000.0f

struct Vec;
struct Rect;
struct Buffer;
struct Chunk;
struct Batch;
struct Node;
class Updater;
class Renderer;
class ShaderManager;

template <typename Type, int PageSize> class AllocatorPage
{
public:
    // The memory used by this allocator
    char data[sizeof(Type) * PageSize];

    // 'blocks' contains a list of free indices which can be allocated.
    // The first available index is found in PageSize - available.
    int blocks[PageSize];

    // 'available' is the number of available instances this page has left to allocate.
    int available;

    // This is not strictly needed, but useful for sanity checking and anyway
    // pretty small..
    QBitArray allocated;

    AllocatorPage()
        : available(PageSize)
        , allocated(PageSize)
    {
        for (int i=0; i<PageSize; ++i) blocks[i] = i;
    }

    const Type *at(uint index) const
    {
        return (Type *) &data[index * sizeof(Type)];
    }

    Type *at(uint index)
    {
        return (Type *) &data[index * sizeof(Type)];
    }
};

template <typename Type, int PageSize> class Allocator
{
public:
    Allocator()
    {
        pages.push_back(new AllocatorPage<Type, PageSize>());
    }

    ~Allocator()
    {
        qDeleteAll(pages);
    }

    Type *allocate()
    {
        AllocatorPage<Type, PageSize> *p = 0;
        for (int i=0; i<pages.size(); ++i) {
            if (pages.at(i)->available > 0) {
                p = pages.at(i);
                break;
            }
        }
        if (!p) {
            p = new AllocatorPage<Type, PageSize>();
            pages.push_back(p);
        }
        uint pos = p->blocks[PageSize - p->available];
        void *mem = p->at(pos);
        p->available--;
        p->allocated.setBit(pos);
        Type *t = new (mem) Type();
        return t;
    }

    void releaseExplicit(uint pageIndex, uint index)
    {
        AllocatorPage<Type, PageSize> *page = pages.at(pageIndex);
        if (!page->allocated.testBit(index))
            qFatal("Double delete in allocator: page=%d, index=%d", pageIndex , index);

        // Call the destructor
        page->at(index)->~Type();

        page->allocated[index] = false;
        page->available++;
        page->blocks[PageSize - page->available] = index;

        // Remove the pages if they are empty and they are the last ones. We need to keep the
        // order of pages since we have references to their index, so we can only remove
        // from the end.
        while (page->available == PageSize && pages.size() > 1 && pages.back() == page) {
            pages.pop_back();
            delete page;
            page = pages.back();
        }
    }

    void release(Type *t)
    {
        int pageIndex = -1;
        for (int i=0; i<pages.size(); ++i) {
            AllocatorPage<Type, PageSize> *p = pages.at(i);
            if ((Type *) (&p->data[0]) <= t && (Type *) (&p->data[PageSize * sizeof(Type)]) > t) {
                pageIndex = i;
                break;
            }
        }
        Q_ASSERT(pageIndex >= 0);

        AllocatorPage<Type, PageSize> *page = pages.at(pageIndex);
        int index = (quint64(t) - quint64(&page->data[0])) / sizeof(Type);

        releaseExplicit(pageIndex, index);
    }

    QVector<AllocatorPage<Type, PageSize> *> pages;
};


inline bool hasMaterialWithBlending(QSGGeometryNode *n)
{
    return (n->opaqueMaterial() ? n->opaqueMaterial()->flags() & QSGMaterial::Blending
                                : n->material()->flags() & QSGMaterial::Blending);
}

struct Pt {
    float x, y;

    void map(const QMatrix4x4 &mat) {
        Pt r;
        const float *m = mat.constData();
        r.x = x * m[0] + y * m[4] + m[12];
        r.y = x * m[1] + y * m[5] + m[13];
        x = r.x;
        y = r.y;
    }

    void set(float nx, float ny) {
        x = nx;
        y = ny;
    }
};

inline QDebug operator << (QDebug d, const Pt &p) {
    d << "Pt(" << p.x << p.y << ")";
    return d;
}



struct Rect {
    Pt tl, br; // Top-Left (min) and Bottom-Right (max)

    void operator |= (const Pt &pt) {
        if (pt.x < tl.x)
            tl.x = pt.x;
        if (pt.x > br.x)
            br.x = pt.x;
        if (pt.y < tl.y)
            tl.y = pt.y;
        if (pt.y > br.y)
            br.y = pt.y;
    }

    void operator |= (const Rect &r) {
        if (r.tl.x < tl.x)
            tl.x = r.tl.x;
        if (r.tl.y < tl.y)
            tl.y = r.tl.y;
        if (r.br.x > br.x)
            br.x = r.br.x;
        if (r.br.y > br.y)
            br.y = r.br.y;
    }

    void map(const QMatrix4x4 &m);

    void set(float left, float top, float right, float bottom) {
        tl.set(left, top);
        br.set(right, bottom);
    }

    bool intersects(const Rect &r) {
        bool xOverlap = r.tl.x < br.x && r.br.x > tl.x;
        bool yOverlap = r.tl.y < br.y && r.br.y > tl.y;
        return xOverlap && yOverlap;
    }

    bool isOutsideFloatRange() const {
        return tl.x < -QSG_RENDERER_COORD_LIMIT
                || tl.y < -QSG_RENDERER_COORD_LIMIT
                || br.x > QSG_RENDERER_COORD_LIMIT
                || br.y > QSG_RENDERER_COORD_LIMIT;
    }
};

inline QDebug operator << (QDebug d, const Rect &r) {
    d << "Rect(" << r.tl.x << r.tl.y << r.br.x << r.br.y << ")";
    return d;
}

struct Buffer {
    GLuint id;
    int size;
    // Data is only valid while preparing the upload. Exception is if we are using the
    // broken IBO workaround or we are using a visualization mode.
    char *data;
};

struct Element {

    Element()
        : node(0)
        , batch(0)
        , nextInBatch(0)
        , root(0)
        , order(0)
        , boundsComputed(false)
        , boundsOutsideFloatRange(false)
        , translateOnlyToRoot(false)
        , removed(false)
        , orphaned(false)
        , isRenderNode(false)
        , isMaterialBlended(false)
    {
    }

    void setNode(QSGGeometryNode *n) {
        node = n;
        isMaterialBlended = hasMaterialWithBlending(n);
    }

    inline void ensureBoundsValid() {
        if (!boundsComputed)
            computeBounds();
    }
    void computeBounds();

    QSGGeometryNode *node;
    Batch *batch;
    Element *nextInBatch;
    Node *root;

    Rect bounds; // in device coordinates

    int order;

    uint boundsComputed : 1;
    uint boundsOutsideFloatRange : 1;
    uint translateOnlyToRoot : 1;
    uint removed : 1;
    uint orphaned : 1;
    uint isRenderNode : 1;
    uint isMaterialBlended : 1;
};

struct RenderNodeElement : public Element {

    RenderNodeElement(QSGRenderNode *rn)
        : renderNode(rn)
    {
        isRenderNode = true;
    }

    QSGRenderNode *renderNode;
};

struct BatchRootInfo {
    BatchRootInfo() : parentRoot(0), lastOrder(-1), firstOrder(-1), availableOrders(0) { }
    QSet<Node *> subRoots;
    Node *parentRoot;
    int lastOrder;
    int firstOrder;
    int availableOrders;
};

struct ClipBatchRootInfo : public BatchRootInfo
{
    QMatrix4x4 matrix;
};

struct DrawSet
{
    DrawSet(int v, int z, int i)
        : vertices(v)
        , zorders(z)
        , indices(i)
        , indexCount(0)
    {
    }
    DrawSet() : vertices(0), zorders(0), indices(0), indexCount(0) {}
    int vertices;
    int zorders;
    int indices;
    int indexCount;
};

enum BatchCompatibility
{
    BatchBreaksOnCompare,
    BatchIsCompatible
};

struct Batch
{
    Batch() : drawSets(1) {}
    bool geometryWasChanged(QSGGeometryNode *gn);
    BatchCompatibility isMaterialCompatible(Element *e) const;
    void invalidate();
    void cleanupRemovedElements();

    bool isTranslateOnlyToRoot() const;
    bool isSafeToBatch() const;

    // pseudo-constructor...
    void init() {
        first = 0;
        root = 0;
        vertexCount = 0;
        indexCount = 0;
        isOpaque = false;
        needsUpload = false;
        merged = false;
        positionAttribute = -1;
        uploadedThisFrame = false;
        isRenderNode = false;
    }

    Element *first;
    Node *root;

    int positionAttribute;

    int vertexCount;
    int indexCount;

    int lastOrderInBatch;

    uint isOpaque : 1;
    uint needsUpload : 1;
    uint merged : 1;
    uint isRenderNode : 1;

    mutable uint uploadedThisFrame : 1; // solely for debugging purposes

    Buffer vbo;
#ifdef QSG_SEPARATE_INDEX_BUFFER
    Buffer ibo;
#endif

    QDataBuffer<DrawSet> drawSets;
};

struct Node
{
    Node()
        : sgNode(0)
        , parent(0)
        , data(0)
        , dirtyState(0)
        , isOpaque(false)
        , isBatchRoot(false)
    {
    }

    QSGNode *sgNode;
    Node *parent;
    void *data;
    QList<Node *> children;

    QSGNode::DirtyState dirtyState;

    uint isOpaque : 1;
    uint isBatchRoot : 1;
    uint becameBatchRoot : 1;

    inline QSGNode::NodeType type() const { return sgNode->type(); }

    inline Element *element() const {
        Q_ASSERT(sgNode->type() == QSGNode::GeometryNodeType);
        return (Element *) data;
    }

    inline RenderNodeElement *renderNodeElement() const {
        Q_ASSERT(sgNode->type() == QSGNode::RenderNodeType);
        return (RenderNodeElement *) data;
    }

    inline ClipBatchRootInfo *clipInfo() const {
        Q_ASSERT(sgNode->type() == QSGNode::ClipNodeType);
        return (ClipBatchRootInfo *) data;
    }

    inline BatchRootInfo *rootInfo() const {
        Q_ASSERT(sgNode->type() == QSGNode::ClipNodeType
                 || (sgNode->type() == QSGNode::TransformNodeType && isBatchRoot));
        return (BatchRootInfo *) data;
    }
};

class Updater : public QSGNodeUpdater
{
public:
    Updater(Renderer *r);

    void visitOpacityNode(Node *n);
    void visitTransformNode(Node *n);
    void visitGeometryNode(Node *n);
    void visitClipNode(Node *n);
    void updateRootTransforms(Node *n);
    void updateRootTransforms(Node *n, Node *root, const QMatrix4x4 &combined);

    void updateStates(QSGNode *n);
    void visitNode(Node *n);
    void registerWithParentRoot(QSGNode *subRoot, QSGNode *parentRoot);

private:
    Renderer *renderer;

    QDataBuffer<Node *> m_roots;
    QDataBuffer<QMatrix4x4> m_rootMatrices;

    int m_added;
    int m_transformChange;
    int m_opacityChange;

    QMatrix4x4 m_identityMatrix;
};

class ShaderManager : public QObject
{
    Q_OBJECT
public:
    struct Shader {
        ~Shader() { delete program; }
        int id_zRange;
        int pos_order;
        QSGMaterialShader *program;

        float lastOpacity;
    };

    ShaderManager(QSGRenderContext *ctx) : blitProgram(0), visualizeProgram(0), context(ctx) { }
    ~ShaderManager() {
        qDeleteAll(rewrittenShaders.values());
        qDeleteAll(stockShaders.values());
    }

public Q_SLOTS:
    void invalidated();

public:
    Shader *prepareMaterial(QSGMaterial *material);
    Shader *prepareMaterialNoRewrite(QSGMaterial *material);

    QHash<QSGMaterialType *, Shader *> rewrittenShaders;
    QHash<QSGMaterialType *, Shader *> stockShaders;

    QOpenGLShaderProgram *blitProgram;
    QOpenGLShaderProgram *visualizeProgram;
    QSGRenderContext *context;
};

class Q_QUICK_PRIVATE_EXPORT Renderer : public QSGRenderer, public QOpenGLFunctions
{
public:
    Renderer(QSGRenderContext *);
    ~Renderer();

    enum VisualizeMode {
        VisualizeNothing,
        VisualizeBatches,
        VisualizeClipping,
        VisualizeChanges,
        VisualizeOverdraw
    };

protected:
    void nodeChanged(QSGNode *node, QSGNode::DirtyState state);
    void render();

private:
    enum ClipTypeBit
    {
        NoClip = 0x00,
        ScissorClip = 0x01,
        StencilClip = 0x02
    };
    Q_DECLARE_FLAGS(ClipType, ClipTypeBit)

    enum RebuildFlag {
        BuildRenderListsForTaggedRoots      = 0x0001,
        BuildRenderLists                    = 0x0002,
        BuildBatches                        = 0x0004,
        FullRebuild                         = 0xffff
    };

    friend class Updater;

    void map(Buffer *buffer, int size, bool isIndexBuf = false);
    void unmap(Buffer *buffer, bool isIndexBuf = false);

    void buildRenderListsFromScratch();
    void buildRenderListsForTaggedRoots();
    void tagSubRoots(Node *node);
    void buildRenderLists(QSGNode *node);

    void deleteRemovedElements();
    void cleanupBatches(QDataBuffer<Batch *> *batches);
    void prepareOpaqueBatches();
    bool checkOverlap(int first, int last, const Rect &bounds);
    void prepareAlphaBatches();
    void invalidateBatchAndOverlappingRenderOrders(Batch *batch);

    void uploadBatch(Batch *b);
    void uploadMergedElement(Element *e, int vaOffset, char **vertexData, char **zData, char **indexData, quint16 *iBase, int *indexCount);

    void renderBatches();
    void renderMergedBatch(const Batch *batch);
    void renderUnmergedBatch(const Batch *batch);
    ClipType updateStencilClip(const QSGClipNode *clip);
    void updateClip(const QSGClipNode *clipList, const Batch *batch);
    const QMatrix4x4 &matrixForRoot(Node *node);
    void renderRenderNode(Batch *batch);
    void setActiveShader(QSGMaterialShader *program, ShaderManager::Shader *shader);

    bool changeBatchRoot(Node *node, Node *newRoot);
    void registerBatchRoot(Node *childRoot, Node *parentRoot);
    void removeBatchRootFromParent(Node *childRoot);
    void nodeChangedBatchRoot(Node *node, Node *root);
    void turnNodeIntoBatchRoot(Node *node);
    void nodeWasTransformed(Node *node, int *vertexCount);
    void nodeWasRemoved(Node *node);
    void nodeWasAdded(QSGNode *node, Node *shadowParent);
    BatchRootInfo *batchRootInfo(Node *node);
    void updateLineWidth(QSGGeometry *g);

    inline Batch *newBatch();
    void invalidateAndRecycleBatch(Batch *b);

    void visualize();
    void visualizeBatch(Batch *b);
    void visualizeClipping(QSGNode *node);
    void visualizeChangesPrepare(Node *n, uint parentChanges = 0);
    void visualizeChanges(Node *n);
    void visualizeOverdraw();
    void visualizeOverdraw_helper(Node *node);
    void visualizeDrawGeometry(const QSGGeometry *g);
    void setCustomRenderMode(const QByteArray &mode);

    QSet<Node *> m_taggedRoots;
    QDataBuffer<Element *> m_opaqueRenderList;
    QDataBuffer<Element *> m_alphaRenderList;
    int m_nextRenderOrder;
    bool m_partialRebuild;
    QSGNode *m_partialRebuildRoot;

    bool m_useDepthBuffer;

    QHash<QSGRenderNode *, RenderNodeElement *> m_renderNodeElements;
    QDataBuffer<Batch *> m_opaqueBatches;
    QDataBuffer<Batch *> m_alphaBatches;
    QHash<QSGNode *, Node *> m_nodes;

    QDataBuffer<Batch *> m_batchPool;
    QDataBuffer<Element *> m_elementsToDelete;
    QDataBuffer<Element *> m_tmpAlphaElements;
    QDataBuffer<Element *> m_tmpOpaqueElements;

    uint m_rebuild;
    qreal m_zRange;
    int m_renderOrderRebuildLower;
    int m_renderOrderRebuildUpper;

    GLuint m_bufferStrategy;
    int m_batchNodeThreshold;
    int m_batchVertexThreshold;

    // Stuff used during rendering only...
    ShaderManager *m_shaderManager;
    QSGMaterial *m_currentMaterial;
    QSGMaterialShader *m_currentProgram;
    ShaderManager::Shader *m_currentShader;

    QRect m_currentScissorRect;
    int m_currentStencilValue;
    QOpenGLShaderProgram m_clipProgram;
    int m_clipMatrixId;
    const QSGClipNode *m_currentClip;
    ClipType m_currentClipType;

    QDataBuffer<char> m_vertexUploadPool;
#ifdef QSG_SEPARATE_INDEX_BUFFER
    QDataBuffer<char> m_indexUploadPool;
#endif
    // For minimal OpenGL core profile support
    QOpenGLVertexArrayObject *m_vao;

    QHash<Node *, uint> m_visualizeChanceSet;
    VisualizeMode m_visualizeMode;

    Allocator<Node, 256> m_nodeAllocator;
    Allocator<Element, 64> m_elementAllocator;
};

Batch *Renderer::newBatch()
{
    Batch *b;
    int size = m_batchPool.size();
    if (size) {
        b = m_batchPool.at(size - 1);
        m_batchPool.resize(size - 1);
    } else {
        b = new Batch();
        memset(&b->vbo, 0, sizeof(Buffer));
#ifdef QSG_SEPARATE_INDEX_BUFFER
        memset(&b->ibo, 0, sizeof(Buffer));
#endif
    }
    b->init();
    return b;
}

}

QT_END_NAMESPACE

#endif // QSGBATCHRENDERER_P_H
