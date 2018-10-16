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

#ifndef FORMLAYOUTMENU
#define FORMLAYOUTMENU

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

#include "shared_global_p.h"
#include <QtCore/QObject>
#include <QtCore/QList>
#include <QtCore/QPointer>

QT_BEGIN_NAMESPACE

class QDesignerFormWindowInterface;

class QAction;
class QWidget;

namespace qdesigner_internal {

// Task menu to be used for form layouts. Offers an options "Add row" which
// pops up a dialog in which the user can specify label name, text and buddy.
class QDESIGNER_SHARED_EXPORT FormLayoutMenu : public QObject
{
    Q_DISABLE_COPY(FormLayoutMenu)
    Q_OBJECT
public:
    typedef QList<QAction *> ActionList;

    explicit FormLayoutMenu(QObject *parent);

    // Populate a list of actions with the form layout actions.
    void populate(QWidget *w, QDesignerFormWindowInterface *fw, ActionList &actions);
    // For implementing QDesignerTaskMenuExtension::preferredEditAction():
    // Return appropriate action for double clicking.
    QAction *preferredEditAction(QWidget *w, QDesignerFormWindowInterface *fw);

private slots:
    void slotAddRow();

private:
    QAction *m_separator1;
    QAction *m_populateFormAction;
    QAction *m_separator2;
    QPointer<QWidget> m_widget;
};
}  // namespace qdesigner_internal

QT_END_NAMESPACE

#endif // FORMLAYOUTMENU
