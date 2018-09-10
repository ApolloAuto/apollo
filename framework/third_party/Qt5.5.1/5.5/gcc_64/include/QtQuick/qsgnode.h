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

#ifndef QSGNODE_H
#define QSGNODE_H

#include <QtQuick/qsggeometry.h>
#include <QtGui/QMatrix4x4>

#include <float.h>

QT_BEGIN_NAMESPACE

#ifndef QT_NO_DEBUG
#define QSG_RUNTIME_DESCRIPTION
#endif

class QSGAbstractRenderer;
class QSGRenderer;

class QSGNode;
class QSGRootNode;
class QSGGeometryNode;
class QSGTransformNode;
class QSGClipNode;
class QSGNodePrivate;
class QSGBasicGeometryNodePrivate;
class QSGGeometryNodePrivate;

namespace QSGBatchRenderer {
    class Renderer;
    class Updater;
}

class Q_QUICK_EXPORT QSGNode
{
public:
    enum NodeType {
        BasicNodeType,
        GeometryNodeType,
        TransformNodeType,
        ClipNodeType,
        OpacityNodeType,
#ifndef qdoc
        RootNodeType,
        RenderNodeType
#endif
    };

    enum Flag {
        // Lower 16 bites reserved for general node
        OwnedByParent               = 0x0001,
        UsePreprocess               = 0x0002,

        // 0x00ff0000 bits reserved for node subclasses

        // QSGBasicGeometryNode
        OwnsGeometry                = 0x00010000,
        OwnsMaterial                = 0x00020000,
        OwnsOpaqueMaterial          = 0x00040000,

        // Uppermost 8 bits are reserved for internal use.
#ifndef qdoc
        IsVisitableNode             = 0x01000000
#else
        InternalReserved            = 0x01000000
#endif
    };
    Q_DECLARE_FLAGS(Flags, Flag)

    enum DirtyStateBit {
        DirtySubtreeBlocked         = 0x0080,
        DirtyMatrix                 = 0x0100,
        DirtyNodeAdded              = 0x0400,
        DirtyNodeRemoved            = 0x0800,
        DirtyGeometry               = 0x1000,
        DirtyMaterial               = 0x2000,
        DirtyOpacity                = 0x4000,

#ifndef qdoc
        DirtyForceUpdate            = 0x8000,

        DirtyUsePreprocess          = UsePreprocess,

        DirtyPropagationMask        = DirtyMatrix
                                      | DirtyNodeAdded
                                      | DirtyOpacity
                                      | DirtyForceUpdate
#endif

    };
    Q_DECLARE_FLAGS(DirtyState, DirtyStateBit)

    QSGNode();
    virtual ~QSGNode();

    QSGNode *parent() const { return m_parent; }

    void removeChildNode(QSGNode *node);
    void removeAllChildNodes();
    void prependChildNode(QSGNode *node);
    void appendChildNode(QSGNode *node);
    void insertChildNodeBefore(QSGNode *node, QSGNode *before);
    void insertChildNodeAfter(QSGNode *node, QSGNode *after);
    void reparentChildNodesTo(QSGNode *newParent);

    int childCount() const;
    QSGNode *childAtIndex(int i) const;
    QSGNode *firstChild() const { return m_firstChild; }
    QSGNode *lastChild() const { return m_lastChild; }
    QSGNode *nextSibling() const { return m_nextSibling; }
    QSGNode* previousSibling() const { return m_previousSibling; }

    inline NodeType type() const { return m_type; }

    QT_DEPRECATED void clearDirty() { }
    void markDirty(DirtyState bits);
    QT_DEPRECATED DirtyState dirtyState() const { return 0; }

    virtual bool isSubtreeBlocked() const;

    Flags flags() const { return m_nodeFlags; }
    void setFlag(Flag, bool = true);
    void setFlags(Flags, bool = true);

    virtual void preprocess() { }

protected:
    QSGNode(NodeType type);
    QSGNode(QSGNodePrivate &dd, NodeType type);

private:
    friend class QSGRootNode;
    friend class QSGBatchRenderer::Renderer;
    friend class QSGRenderer;

    void init();
    void destroy();

    QSGNode *m_parent;
    NodeType m_type;
    QSGNode *m_firstChild;
    QSGNode *m_lastChild;
    QSGNode *m_nextSibling;
    QSGNode *m_previousSibling;
    int m_subtreeRenderableCount;

    Flags m_nodeFlags;
    DirtyState m_dirtyState; // Obsolete, remove in Qt 6

protected:
    friend class QSGNodePrivate;

    QScopedPointer<QSGNodePrivate> d_ptr;
};

void Q_QUICK_EXPORT qsgnode_set_description(QSGNode *node, const QString &description);

class Q_QUICK_EXPORT QSGBasicGeometryNode : public QSGNode
{
public:
    ~QSGBasicGeometryNode();

    void setGeometry(QSGGeometry *geometry);
    const QSGGeometry *geometry() const { return m_geometry; }
    QSGGeometry *geometry() { return m_geometry; }

    const QMatrix4x4 *matrix() const { return m_matrix; }
    const QSGClipNode *clipList() const { return m_clip_list; }

protected:
    QSGBasicGeometryNode(NodeType type);
    QSGBasicGeometryNode(QSGBasicGeometryNodePrivate &dd, NodeType type);

private:
    friend class QSGNodeUpdater;
    friend class QSGBatchRenderer::Updater;

    QSGGeometry *m_geometry;

    int m_reserved_start_index;
    int m_reserved_end_index;

    const QMatrix4x4 *m_matrix;
    const QSGClipNode *m_clip_list;
};

class QSGMaterial;

class Q_QUICK_EXPORT QSGGeometryNode : public QSGBasicGeometryNode
{
public:
    QSGGeometryNode();
    ~QSGGeometryNode();

    void setMaterial(QSGMaterial *material);
    QSGMaterial *material() const { return m_material; }

    void setOpaqueMaterial(QSGMaterial *material);
    QSGMaterial *opaqueMaterial() const { return m_opaque_material; }

    QSGMaterial *activeMaterial() const;

    void setRenderOrder(int order);
    int renderOrder() const { return m_render_order; }

    void setInheritedOpacity(qreal opacity);
    qreal inheritedOpacity() const { return m_opacity; }

protected:
    QSGGeometryNode(QSGGeometryNodePrivate &dd);

private:
    friend class QSGNodeUpdater;

    int m_render_order;
    QSGMaterial *m_material;
    QSGMaterial *m_opaque_material;

    qreal m_opacity;
};

class Q_QUICK_EXPORT QSGClipNode : public QSGBasicGeometryNode
{
public:
    QSGClipNode();
    ~QSGClipNode();

    void setIsRectangular(bool rectHint);
    bool isRectangular() const { return m_is_rectangular; }

    void setClipRect(const QRectF &);
    QRectF clipRect() const { return m_clip_rect; }

private:
    uint m_is_rectangular : 1;
    uint m_reserved : 31;

    QRectF m_clip_rect;
};


class Q_QUICK_EXPORT QSGTransformNode : public QSGNode
{
public:
    QSGTransformNode();
    ~QSGTransformNode();

    void setMatrix(const QMatrix4x4 &matrix);
    const QMatrix4x4 &matrix() const { return m_matrix; }

    void setCombinedMatrix(const QMatrix4x4 &matrix);
    const QMatrix4x4 &combinedMatrix() const { return m_combined_matrix; }

private:
    QMatrix4x4 m_matrix;
    QMatrix4x4 m_combined_matrix;
};


class Q_QUICK_EXPORT QSGRootNode : public QSGNode
{
public:
    QSGRootNode();
    ~QSGRootNode();

private:
    void notifyNodeChange(QSGNode *node, DirtyState state);

    friend class QSGAbstractRenderer;
    friend class QSGNode;
    friend class QSGGeometryNode;

    QList<QSGAbstractRenderer *> m_renderers;
};


class Q_QUICK_EXPORT QSGOpacityNode : public QSGNode
{
public:
    QSGOpacityNode();
    ~QSGOpacityNode();

    void setOpacity(qreal opacity);
    qreal opacity() const { return m_opacity; }

    void setCombinedOpacity(qreal opacity);
    qreal combinedOpacity() const { return m_combined_opacity; }

    bool isSubtreeBlocked() const;

private:
    qreal m_opacity;
    qreal m_combined_opacity;
};

class Q_QUICK_EXPORT QSGNodeVisitor {
public:
    virtual ~QSGNodeVisitor();

protected:
    virtual void enterTransformNode(QSGTransformNode *) {}
    virtual void leaveTransformNode(QSGTransformNode *) {}
    virtual void enterClipNode(QSGClipNode *) {}
    virtual void leaveClipNode(QSGClipNode *) {}
    virtual void enterGeometryNode(QSGGeometryNode *) {}
    virtual void leaveGeometryNode(QSGGeometryNode *) {}
    virtual void enterOpacityNode(QSGOpacityNode *) {}
    virtual void leaveOpacityNode(QSGOpacityNode *) {}
    virtual void visitNode(QSGNode *n);
    virtual void visitChildren(QSGNode *n);
};

#ifndef QT_NO_DEBUG_STREAM
Q_QUICK_EXPORT QDebug operator<<(QDebug, const QSGNode *n);
Q_QUICK_EXPORT QDebug operator<<(QDebug, const QSGGeometryNode *n);
Q_QUICK_EXPORT QDebug operator<<(QDebug, const QSGTransformNode *n);
Q_QUICK_EXPORT QDebug operator<<(QDebug, const QSGOpacityNode *n);
Q_QUICK_EXPORT QDebug operator<<(QDebug, const QSGRootNode *n);

#endif

Q_DECLARE_OPERATORS_FOR_FLAGS(QSGNode::DirtyState)
Q_DECLARE_OPERATORS_FOR_FLAGS(QSGNode::Flags)

QT_END_NAMESPACE

#endif // QSGNODE_H
