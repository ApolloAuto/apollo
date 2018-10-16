/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the Qt Designer of the Qt Toolkit.
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
// This file is not part of the Qt API.  It exists for the convenience
// of Qt Designer.  This header
// file may change from version to version without notice, or even be removed.
//
// We mean it.
//

#ifndef RICHTEXTEDITOR_H
#define RICHTEXTEDITOR_H

#include <QtWidgets/QTextEdit>
#include <QtWidgets/QDialog>
#include "shared_global_p.h"

QT_BEGIN_NAMESPACE

class QTabWidget;
class QToolBar;

class QDesignerFormEditorInterface;

namespace qdesigner_internal {

class RichTextEditor;

class QDESIGNER_SHARED_EXPORT RichTextEditorDialog : public QDialog
{
    Q_OBJECT
public:
    explicit RichTextEditorDialog(QDesignerFormEditorInterface *core, QWidget *parent = 0);
    ~RichTextEditorDialog();

    int showDialog();
    void setDefaultFont(const QFont &font);
    void setText(const QString &text);
    QString text(Qt::TextFormat format = Qt::AutoText) const;

private slots:
    void tabIndexChanged(int newIndex);
    void richTextChanged();
    void sourceChanged();

private:
    enum TabIndex { RichTextIndex, SourceIndex };
    enum State { Clean, RichTextChanged, SourceChanged };
    RichTextEditor *m_editor;
    QTextEdit      *m_text_edit;
    QTabWidget     *m_tab_widget;
    State m_state;
    QDesignerFormEditorInterface *m_core;
    int m_initialTab;
};

} // namespace qdesigner_internal

QT_END_NAMESPACE

#endif // RITCHTEXTEDITOR_H
