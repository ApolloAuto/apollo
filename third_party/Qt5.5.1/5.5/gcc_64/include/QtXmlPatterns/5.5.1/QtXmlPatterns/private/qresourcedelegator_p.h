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

#ifndef QPatternist_ResourceDelegator_p_H
#define QPatternist_ResourceDelegator_p_H

#include <QSet>
#include <QUrl>

#include <private/qdeviceresourceloader_p.h>

QT_BEGIN_NAMESPACE

namespace QPatternist
{
    /**
     * @short Delegates to another ResourceLoader, but in case a URI is in an
     * exception list, it delegates to a different loader.
     *
     * This is used for handling device variables, since when a device variable
     * is rebound, a resource loader needs to carry that binding, while the
     * resource loader for the other query remains as is.
     *
     * @since 4.5
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class ResourceDelegator : public DeviceResourceLoader
    {
    public:
        ResourceDelegator(const QSet<QUrl> &needsOverride,
                          const ResourceLoader::Ptr &parentLoader,
                          const ResourceLoader::Ptr &forDeviceLoader) : m_needsOverride(needsOverride)
                                                                      , m_parentLoader(parentLoader)
                                                                      , m_forDeviceLoader(forDeviceLoader)

        {
            Q_ASSERT(m_parentLoader);
        }

        virtual bool isUnparsedTextAvailable(const QUrl &uri,
                                             const QString &encoding);
        virtual ItemType::Ptr announceUnparsedText(const QUrl &uri);
        virtual Item openUnparsedText(const QUrl &uri,
                                      const QString &encoding,
                                      const ReportContext::Ptr &context,
                                      const SourceLocationReflection *const where);
        virtual Item openDocument(const QUrl &uri,
                                  const ReportContext::Ptr &context);
        virtual SequenceType::Ptr announceDocument(const QUrl &uri, const Usage usageHint);
        virtual bool isDocumentAvailable(const QUrl &uri);
        virtual Item::Iterator::Ptr openCollection(const QUrl &uri);
        virtual SequenceType::Ptr announceCollection(const QUrl &uri);

        /**
         * Returns the union of the deviceURIs() that ResourceDelegator's two
         * resource loaders has.
         */
        virtual QSet<QUrl> deviceURIs() const;

    private:
        const QSet<QUrl> m_needsOverride;
        const ResourceLoader::Ptr m_parentLoader;
        const ResourceDelegator::Ptr m_forDeviceLoader;
    };
}

QT_END_NAMESPACE

#endif
