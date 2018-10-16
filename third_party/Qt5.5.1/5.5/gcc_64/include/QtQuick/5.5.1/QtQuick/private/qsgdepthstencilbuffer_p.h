/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the Qt Quick module of the Qt Toolkit.
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

#ifndef QSGDEPTHSTENCILBUFFER_P_H
#define QSGDEPTHSTENCILBUFFER_P_H

#include <QtCore/qsize.h>
#include <QtGui/private/qopenglcontext_p.h>
#include <QtGui/private/qopenglextensions_p.h>
#include <QtCore/qsharedpointer.h>
#include <QtCore/qhash.h>

QT_BEGIN_NAMESPACE

class QSGDepthStencilBufferManager;

class QSGDepthStencilBuffer
{
public:
    enum Attachment
    {
        NoAttachment = 0x00,
        DepthAttachment = 0x01,
        StencilAttachment = 0x02
    };
    Q_DECLARE_FLAGS(Attachments, Attachment)

    struct Format
    {
        QSize size;
        int samples;
        QSGDepthStencilBuffer::Attachments attachments;
        bool operator == (const Format &other) const;
    };

    QSGDepthStencilBuffer(QOpenGLContext *context, const Format &format);
    virtual ~QSGDepthStencilBuffer();

    // Attaches this depth stencil buffer to the currently bound FBO.
    void attach();
    // Detaches this depth stencil buffer from the currently bound FBO.
    void detach();

    QSize size() const { return m_format.size; }
    int samples() const { return m_format.samples; }
    Attachments attachments() const { return m_format.attachments; }

protected:
    virtual void free() = 0;

    QOpenGLExtensions m_functions;
    QSGDepthStencilBufferManager *m_manager;
    Format m_format;
    GLuint m_depthBuffer;
    GLuint m_stencilBuffer;

    friend class QSGDepthStencilBufferManager;
};

Q_DECLARE_OPERATORS_FOR_FLAGS(QSGDepthStencilBuffer::Attachments)

inline bool QSGDepthStencilBuffer::Format::operator == (const Format &other) const
{
    return size == other.size && samples == other.samples && attachments == other.attachments;
}


class QSGDefaultDepthStencilBuffer : public QSGDepthStencilBuffer
{
public:
    QSGDefaultDepthStencilBuffer(QOpenGLContext *context, const Format &format);
    virtual ~QSGDefaultDepthStencilBuffer();

protected:
    virtual void free();
};


class QSGDepthStencilBufferManager
{
public:
    QSGDepthStencilBufferManager(QOpenGLContext *ctx) : m_context(ctx) { }
    ~QSGDepthStencilBufferManager();
    QOpenGLContext *context() const { return m_context; }
    QSharedPointer<QSGDepthStencilBuffer> bufferForFormat(const QSGDepthStencilBuffer::Format &fmt);
    void insertBuffer(const QSharedPointer<QSGDepthStencilBuffer> &buffer);

private:
    typedef QHash<QSGDepthStencilBuffer::Format, QWeakPointer<QSGDepthStencilBuffer> > Hash;
    QOpenGLContext *m_context;
    Hash m_buffers;

    friend class QSGDepthStencilBuffer;
};

extern uint qHash(const QSGDepthStencilBuffer::Format &format);

QT_END_NAMESPACE

#endif
