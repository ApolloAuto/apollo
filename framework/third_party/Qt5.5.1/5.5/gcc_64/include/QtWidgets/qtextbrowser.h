/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the QtWidgets module of the Qt Toolkit.
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

#ifndef QTEXTBROWSER_H
#define QTEXTBROWSER_H

#include <QtWidgets/qtextedit.h>
#include <QtCore/qurl.h>

QT_BEGIN_NAMESPACE


#ifndef QT_NO_TEXTBROWSER

class QTextBrowserPrivate;

class Q_WIDGETS_EXPORT QTextBrowser : public QTextEdit
{
    Q_OBJECT

    Q_PROPERTY(QUrl source READ source WRITE setSource)
    Q_OVERRIDE(bool modified SCRIPTABLE false)
    Q_OVERRIDE(bool readOnly DESIGNABLE false SCRIPTABLE false)
    Q_OVERRIDE(bool undoRedoEnabled DESIGNABLE false SCRIPTABLE false)
    Q_PROPERTY(QStringList searchPaths READ searchPaths WRITE setSearchPaths)
    Q_PROPERTY(bool openExternalLinks READ openExternalLinks WRITE setOpenExternalLinks)
    Q_PROPERTY(bool openLinks READ openLinks WRITE setOpenLinks)

public:
    explicit QTextBrowser(QWidget* parent = 0);
    virtual ~QTextBrowser();

    QUrl source() const;

    QStringList searchPaths() const;
    void setSearchPaths(const QStringList &paths);

    virtual QVariant loadResource(int type, const QUrl &name) Q_DECL_OVERRIDE;

    bool isBackwardAvailable() const;
    bool isForwardAvailable() const;
    void clearHistory();
    QString historyTitle(int) const;
    QUrl historyUrl(int) const;
    int backwardHistoryCount() const;
    int forwardHistoryCount() const;

    bool openExternalLinks() const;
    void setOpenExternalLinks(bool open);

    bool openLinks() const;
    void setOpenLinks(bool open);

public Q_SLOTS:
    virtual void setSource(const QUrl &name);
    virtual void backward();
    virtual void forward();
    virtual void home();
    virtual void reload();

Q_SIGNALS:
    void backwardAvailable(bool);
    void forwardAvailable(bool);
    void historyChanged();
    void sourceChanged(const QUrl &);
    void highlighted(const QUrl &);
    void highlighted(const QString &);
    void anchorClicked(const QUrl &);

protected:
    bool event(QEvent *e) Q_DECL_OVERRIDE;
    virtual void keyPressEvent(QKeyEvent *ev) Q_DECL_OVERRIDE;
    virtual void mouseMoveEvent(QMouseEvent *ev) Q_DECL_OVERRIDE;
    virtual void mousePressEvent(QMouseEvent *ev) Q_DECL_OVERRIDE;
    virtual void mouseReleaseEvent(QMouseEvent *ev) Q_DECL_OVERRIDE;
    virtual void focusOutEvent(QFocusEvent *ev) Q_DECL_OVERRIDE;
    virtual bool focusNextPrevChild(bool next) Q_DECL_OVERRIDE;
    virtual void paintEvent(QPaintEvent *e) Q_DECL_OVERRIDE;

private:
    Q_DISABLE_COPY(QTextBrowser)
    Q_DECLARE_PRIVATE(QTextBrowser)
    Q_PRIVATE_SLOT(d_func(), void _q_documentModified())
    Q_PRIVATE_SLOT(d_func(), void _q_activateAnchor(const QString &))
    Q_PRIVATE_SLOT(d_func(), void _q_highlightLink(const QString &))
};

#endif // QT_NO_TEXTBROWSER

QT_END_NAMESPACE

#endif // QTEXTBROWSER_H
