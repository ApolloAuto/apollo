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

#ifndef PROMOTIONMODEL_H
#define PROMOTIONMODEL_H

#include <QtGui/QStandardItemModel>
#include <QtCore/QSet>

QT_BEGIN_NAMESPACE

class QDesignerFormEditorInterface;
class QDesignerWidgetDataBaseItemInterface;

namespace qdesigner_internal {

    // Item model representing the promoted widgets.
    class PromotionModel : public QStandardItemModel {
        Q_OBJECT

    public:
        explicit PromotionModel(QDesignerFormEditorInterface *core);

        void updateFromWidgetDatabase();

        // Return item at model index or 0.
        QDesignerWidgetDataBaseItemInterface *databaseItemAt(const QModelIndex &, bool *referenced) const;

        QModelIndex indexOfClass(const QString &className) const;

   signals:
        void includeFileChanged(QDesignerWidgetDataBaseItemInterface *, const QString &includeFile);
        void classNameChanged(QDesignerWidgetDataBaseItemInterface *, const QString &newName);

    private slots:
        void slotItemChanged(QStandardItem * item);

    private:
        void initializeHeaders();
        // Retrieve data base item of item or return 0.
        QDesignerWidgetDataBaseItemInterface *databaseItem(const QStandardItem * item, bool *referenced) const;

        QDesignerFormEditorInterface *m_core;
    };
} // namespace qdesigner_internal

QT_END_NAMESPACE

#endif // PROMOTIONMODEL_H
