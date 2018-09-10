/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the QtXmlPatterns module of the Qt Toolkit.
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

//
//  W A R N I N G
//  -------------
//
// This file is not part of the Qt API.  It exists purely as an
// implementation detail.  This header file may change from version to
// version without notice, or even be removed.
//
// We mean it.

#ifndef Patternist_AccelTree_H
#define Patternist_AccelTree_H

#include <QHash>
#include <QUrl>
#include <QVector>
#include <QXmlName>

#include <private/qitem_p.h>
#include <private/qnamepool_p.h>

QT_BEGIN_NAMESPACE

namespace QPatternist
{
    template<bool> class AccelTreeBuilder;

    /**
     * @short Stores an XML document using the XPath Accelerator scheme, also
     * known as pre/post numbering.
     *
     * Working on this code will be destructive without a proper understanding of
     * the Accelerator scheme, so do check out the links. We don't implement any form
     * of staircase join, although that is only due to time constraints.
     *
     * @author Frans Englich <frans.englich@nokia.com>
     * @see <a href="http://www.pathfinder-xquery.org/?q=research/xpath-accel">XPath
     * Accelerator</a>
     * @see <a href="http://www.pathfinder-xquery.org/files/xpath-accel.pdf">Accelerating
     * XPath Location Steps, Torsten Grust</a>
     * @see <a href="http://citeseer.ist.psu.edu/cache/papers/cs/29367/http:zSzzSzwww.informatik.uni-konstanz.dezSz~grustzSzfileszSzstaircase-join.pdf/grust03staircase.pdf">Staircase Join:
     * Teach a Relational DBMS to Watch its (Axis) Steps</a>
     * @see <a href="http://ftp.cwi.nl/CWIreports/INS/INS-E0510.pdf">Loop-lifted
     * staircase join: from XPath to XQuery, Torsten Grust</a>
     * @see <a href="http://englich.wordpress.com/2007/01/09/xmlstat/">xmlstat, Frans Englich</a>
     * @see <a href"http://www.inf.uni-konstanz.de/dbis/publications/download/accelerating-locsteps.pdf">Accelerating
     * XPath Evaluation in Any RDBMS, Torsten Grust</a>
     */
    class Q_AUTOTEST_EXPORT AccelTree : public QAbstractXmlNodeModel
    {
        friend class AccelTreePrivate;
    public:
        using QAbstractXmlNodeModel::createIndex;

        typedef QExplicitlySharedDataPointer<AccelTree> Ptr;
        typedef qint32 PreNumber;
        typedef PreNumber PostNumber;
        typedef qint8 Depth;

        AccelTree(const QUrl &docURI, const QUrl &bURI);

        /**
         * @short Houses data for a node, and that all node kinds have.
         *
         * BasicNodeData is internal to the Accel tree implementation, and is
         * only used by those classes.
         *
         * @author Frans Englich <frans.englich@nokia.com>
         * @todo Can't m_kind be coded somewhere else? If m_name is invalid,
         * its bits can be used to distinguish the node types that doesn't have
         * names, and for elements, attributes and processing instructions, we need
         * two bits, somewhere. Attributes and processing instructions can't have a
         * size, is that of help? There's also certain rules for the names. For instance,
         * a processing instruction will never have a prefix nor namespace. Neither
         * will an attribute node have a default, non-empty namespace, right?
         * @todo Compress text nodes, add general support for it in Patternist.
         */
        class BasicNodeData
        {
        public:
            /* No need to initialize the members. See AccelTreeBuilder. */
            inline BasicNodeData()
            {
            }

            inline BasicNodeData(const PreNumber aDepth,
                                 const PreNumber aParent,
                                 const QXmlNodeModelIndex::NodeKind k,
                                 const PreNumber s,
                                 const QXmlName n = QXmlName()) : m_parent(aParent)
                                                                , m_size(s)
                                                                , m_name(n)
                                                                , m_depth(aDepth)
                                                                , m_kind(k)
            {
            }

            inline Depth depth() const
            {
                return m_depth;
            }

            inline PreNumber parent() const
            {
                return m_parent;
            }

            /**
             * @see AccelTree::size()
             */
            inline PreNumber size() const
            {
                /* Remember that we use the m_size to signal compression if
                 * we're a text node. */
                if(m_kind == QXmlNodeModelIndex::Text)
                    return 0;
                else
                    return m_size;
            }

            inline void setSize(const PreNumber aSize)
            {
                m_size = aSize;
            }

            inline QXmlNodeModelIndex::NodeKind kind() const
            {
                return m_kind;
            }

            inline QXmlName name() const
            {
                return m_name;
            }

            inline bool isCompressed() const
            {
                Q_ASSERT_X(m_kind == QXmlNodeModelIndex::Text, Q_FUNC_INFO,
                           "Currently, only text nodes are compressed.");
                /* Note, we don't call size() here, since it has logic for text
                 * nodes. */
                return m_size == IsCompressed;
            }

        private:
            /**
             * This is the pre number of the parent.
             */
            PreNumber                       m_parent;

            /**
             * This is the count of children this node has.
             *
             * In the case of a text node, which cannot have children,
             * it is set to IsCompressed, if the content has been the result
             * of CompressedWhitespace::compress(). If it's not compressed,
             * it is zero.
             */
            PreNumber                       m_size;

            /**
             * For text nodes, and less importantly, comments,
             * this variable is not used.
             */
            QXmlName                        m_name;

            Depth                           m_depth;

            /**
             * Technically it is sufficient with 7 bits. However, at least MSVC
             * 2005 miscompiles it such that QXmlNodeModelIndex::Text becomes
             * -64 instead of 64 with hilarious crashes as result.
             *
             * Fortunately this extra bit would be padded anyway.
             */
            QXmlNodeModelIndex::NodeKind    m_kind : 8;
        };

        virtual QUrl baseUri(const QXmlNodeModelIndex &ni) const;
        virtual QUrl documentUri(const QXmlNodeModelIndex &ni) const;
        virtual QXmlNodeModelIndex::NodeKind kind(const QXmlNodeModelIndex &ni) const;
        virtual QXmlNodeModelIndex::DocumentOrder compareOrder(const QXmlNodeModelIndex &ni1,
                                                               const QXmlNodeModelIndex &ni2) const;

        /**
         * @short Returns the root node.
         *
         * This function does not use @p n, so a default constructed
         * QXmlNodeModelIndex may be passed.
         */
        virtual QXmlNodeModelIndex root(const QXmlNodeModelIndex &n) const;

        virtual QXmlNodeModelIndex parent(const QXmlNodeModelIndex &ni) const;
        virtual QXmlNodeModelIndex::Iterator::Ptr iterate(const QXmlNodeModelIndex &ni,
                                                          QXmlNodeModelIndex::Axis axis) const;
        virtual QXmlName name(const QXmlNodeModelIndex &ni) const;
        virtual QVector<QXmlName> namespaceBindings(const QXmlNodeModelIndex &n) const;
        virtual void sendNamespaces(const QXmlNodeModelIndex &n,
                                    QAbstractXmlReceiver *const receiver) const;
        virtual QString stringValue(const QXmlNodeModelIndex &n) const;
        virtual QVariant typedValue(const QXmlNodeModelIndex &n) const;
        virtual Item::Iterator::Ptr sequencedTypedValue(const QXmlNodeModelIndex &n) const;
        virtual ItemType::Ptr type(const QXmlNodeModelIndex &ni) const;
        virtual QXmlNodeModelIndex elementById(const QXmlName &id) const;
        virtual QVector<QXmlNodeModelIndex> nodesByIdref(const QXmlName &idref) const;
        virtual void copyNodeTo(const QXmlNodeModelIndex &node,
                                QAbstractXmlReceiver *const receiver,
                                const NodeCopySettings &settings) const;

        friend class AccelTreeBuilder<false>;
        friend class AccelTreeBuilder<true>;

        enum Constants
        {
            IsCompressed = 1
        };

        /**
         * The key is the pre number of an element, and the value is a vector
         * containing the namespace declarations being declared on that
         * element. Therefore, it does not reflect the namespaces being in
         * scope for that element. For that, a walk along axis ancestor is
         * necessary.
         */
        QHash<PreNumber, QVector<QXmlName> > namespaces;

        /**
         * Stores data for nodes. The QHash's value is the data of the processing instruction, and the
         * content of a text node or comment.
         */
        QHash<PreNumber, QString> data;

        QVector<BasicNodeData> basicData;
        QHash<PreNumber, QPair<qint64, qint64> > sourcePositions;

        inline QUrl documentUri() const
        {
            return m_documentURI;
        }

        inline QUrl baseUri() const
        {
            return m_baseURI;
        }

        /**
         * @short Returns @c true if the node identified by @p pre has child
         * nodes(in the sense of the XDM), but also if it has namespace nodes,
         * or attribute nodes.
         */
        inline bool hasChildren(const PreNumber pre) const
        {
            return basicData.at(pre).size() > 0;
        }

        /**
         * @short Returns the parent node of @p pre.
         *
         * If @p pre parent doesn't have a parent node, the return value is
         * undefined.
         *
         * @see hasParent()
         */
        inline PreNumber parent(const PreNumber pre) const
        {
            return basicData.at(pre).parent();
        }

        inline bool hasParent(const PreNumber pre) const
        {
            return basicData.at(pre).depth() > 0;
        }

        inline bool hasFollowingSibling(const PreNumber pre) const
        {
            return pre < maximumPreNumber();
        }

        inline PostNumber postNumber(const PreNumber pre) const
        {
            const BasicNodeData &b = basicData.at(pre);
            return pre + b.size() - b.depth();
        }

        inline QXmlNodeModelIndex::NodeKind kind(const PreNumber pre) const
        {
            return basicData.at(pre).kind();
        }

        inline PreNumber maximumPreNumber() const
        {
            return basicData.count() - 1;
        }

        inline PreNumber toPreNumber(const QXmlNodeModelIndex n) const
        {
            return n.data();
        }

        inline PreNumber size(const PreNumber pre) const
        {
            Q_ASSERT_X(basicData.at(pre).size() != -1, Q_FUNC_INFO,
                       "The size cannot be -1. That means an uninitialized value is attempted to be used.");
            return basicData.at(pre).size();
        }

        inline Depth depth(const PreNumber pre) const
        {
            return basicData.at(pre).depth();
        }

        void printStats(const NamePool::Ptr &np) const;

        inline QXmlName name(const PreNumber pre) const
        {
            return basicData.at(pre).name();
        }

        inline bool isCompressed(const PreNumber pre) const
        {
            return basicData.at(pre).isCompressed();
        }

        static inline bool hasPrefix(const QVector<QXmlName> &nbs, const QXmlName::PrefixCode prefix);

        QUrl m_documentURI;
        QUrl m_baseURI;

    protected:
        virtual QXmlNodeModelIndex nextFromSimpleAxis(QAbstractXmlNodeModel::SimpleAxis,
                                                      const QXmlNodeModelIndex&) const;
        virtual QVector<QXmlNodeModelIndex> attributes(const QXmlNodeModelIndex &element) const;

    private:
        /**
         * Returns the source location for the object with the given @p index.
         */
        QSourceLocation sourceLocation(const QXmlNodeModelIndex &index) const;

        /**
         * Copies the children of @p node to @p receiver.
         */
        inline void copyChildren(const QXmlNodeModelIndex &node,
                                 QAbstractXmlReceiver *const receiver,
                                 const NodeCopySettings &settings) const;

        /**
         * The key is the xml:id value, and the value is the element
         * with that value.
         */
        QHash<QXmlName::LocalNameCode, PreNumber> m_IDs;
    };
}

Q_DECLARE_TYPEINFO(QPatternist::AccelTree::BasicNodeData, Q_MOVABLE_TYPE);

QT_END_NAMESPACE

#endif
