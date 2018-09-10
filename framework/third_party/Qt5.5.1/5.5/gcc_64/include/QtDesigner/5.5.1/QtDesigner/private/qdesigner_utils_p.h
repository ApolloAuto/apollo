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

#ifndef QDESIGNER_UTILS_H
#define QDESIGNER_UTILS_H

#include "shared_global_p.h"

#include <QtDesigner/QDesignerFormWindowInterface>

#include <QtCore/QVariant>
#include <QtCore/QSharedDataPointer>
#include <QtCore/QMap>
#include <QtWidgets/QMainWindow>
#include <QtGui/QIcon>
#include <QtGui/QPixmap>

QT_BEGIN_NAMESPACE

class QDebug;

namespace qdesigner_internal {
class QDesignerFormWindowCommand;
class DesignerIconCache;
class FormWindowBase;


QDESIGNER_SHARED_EXPORT void designerWarning(const QString &message);

QDESIGNER_SHARED_EXPORT void reloadIconResources(DesignerIconCache *iconCache, QObject *object);

/* Flag/Enumeration helpers for the property sheet: Enumeration or flag values are returned by the property sheet
 * as a pair of meta type and integer value.
 * The meta type carries all the information required for the property editor and serialization
 * by the form builders (names, etc).
 * Note that the property editor uses unqualified names ("Cancel") while the form builder serialization  (uic)
 * requires the whole string
 * ("QDialogButtonBox::Cancel" or "org.qt-project.qt.gui.QDialogButtonBox.StandardButton.Cancel").*/

/* --------- MetaEnum: Base class representing a QMetaEnum with lookup functions
 * in both ways. Template of int type since unsigned is more suitable for flags.
 * The keyToValue() is ignorant of scopes, it can handle fully qualified or unqualified names. */

template <class IntType>
class MetaEnum
{
public:
    typedef QMap<QString, IntType> KeyToValueMap;

    MetaEnum(const QString &name, const QString &scope, const QString &separator);
    MetaEnum() {}
    void addKey(IntType value, const QString &name);

    QString valueToKey(IntType value, bool *ok = 0) const;
    // Ignorant of scopes.
    IntType keyToValue(QString key, bool *ok = 0) const;

    const QString &name() const      { return m_name; }
    const QString &scope() const     { return m_scope; }
    const QString &separator() const { return m_separator; }

    const QStringList &keys() const { return m_keys; }
    const KeyToValueMap &keyToValueMap() const { return m_keyToValueMap; }

protected:
    void appendQualifiedName(const QString &key, QString &target) const;

private:
    QString m_name;
    QString m_scope;
    QString m_separator;
    KeyToValueMap m_keyToValueMap;
    QStringList m_keys;
};

template <class IntType>
MetaEnum<IntType>::MetaEnum(const QString &name, const QString &scope, const QString &separator) :
    m_name(name),
    m_scope(scope),
    m_separator(separator)
{
}

template <class IntType>
void MetaEnum<IntType>::addKey(IntType value, const QString &name)
{
    m_keyToValueMap.insert(name, value);
    m_keys.append(name);
}

template <class IntType>
QString MetaEnum<IntType>::valueToKey(IntType value, bool *ok) const
{
    const QString rc = m_keyToValueMap.key(value);
    if (ok)
        *ok = !rc.isEmpty();
    return rc;
}

template <class IntType>
IntType MetaEnum<IntType>::keyToValue(QString key, bool *ok) const
{
    if (!m_scope.isEmpty() && key.startsWith(m_scope))
        key.remove(0, m_scope.size() + m_separator.size());
    const typename KeyToValueMap::const_iterator it = m_keyToValueMap.find(key);
    const bool found = it != m_keyToValueMap.constEnd();
    if (ok)
        *ok = found;
    return found ? it.value() : IntType(0);
}

template <class IntType>
void MetaEnum<IntType>::appendQualifiedName(const QString &key, QString &target) const
{
    if (!m_scope.isEmpty()) {
        target += m_scope;
        target += m_separator;
    }
    target += key;
}

// -------------- DesignerMetaEnum: Meta type for enumerations

class QDESIGNER_SHARED_EXPORT DesignerMetaEnum : public MetaEnum<int>
{
public:
    DesignerMetaEnum(const QString &name, const QString &scope, const QString &separator);
    DesignerMetaEnum() {}

    enum SerializationMode { FullyQualified, NameOnly };
    QString toString(int value, SerializationMode sm, bool *ok = 0) const;

    QString messageToStringFailed(int value) const;
    QString messageParseFailed(const QString &s) const;

    // parse a string (ignorant of scopes)
    int parseEnum(const QString &s, bool *ok = 0) const { return keyToValue(s, ok); }
};

// -------------- DesignerMetaFlags: Meta type for flags.
// Note that while the handling of flags is done using unsigned integers, the actual values returned
// by the property system  are integers.

class QDESIGNER_SHARED_EXPORT DesignerMetaFlags : public MetaEnum<uint>
{
public:
    DesignerMetaFlags(const QString &name, const QString &scope, const QString &separator);
    DesignerMetaFlags() {}

    enum SerializationMode { FullyQualified, NameOnly };
    QString toString(int value, SerializationMode sm) const;
    QStringList flags(int value) const;

    QString messageParseFailed(const QString &s) const;
    // parse a string (ignorant of scopes)
    int parseFlags(const QString &s, bool *ok = 0) const;
};

// -------------- EnumValue: Returned by the property sheet for enumerations

struct QDESIGNER_SHARED_EXPORT PropertySheetEnumValue
{
    PropertySheetEnumValue(int v, const DesignerMetaEnum &me);
    PropertySheetEnumValue();

    int value;
    DesignerMetaEnum metaEnum;
};

// -------------- FlagValue: Returned by the property sheet for flags

struct QDESIGNER_SHARED_EXPORT PropertySheetFlagValue
{
    PropertySheetFlagValue(int v, const DesignerMetaFlags &mf);
    PropertySheetFlagValue();

    int value;
    DesignerMetaFlags metaFlags;
};

// -------------- PixmapValue: Returned by the property sheet for pixmaps
class QDESIGNER_SHARED_EXPORT PropertySheetPixmapValue
{
public:
    PropertySheetPixmapValue(const QString &path);
    PropertySheetPixmapValue();

    bool operator==(const PropertySheetPixmapValue &other) const { return compare(other) == 0; }
    bool operator!=(const PropertySheetPixmapValue &other) const { return compare(other) != 0; }
    bool operator<(const PropertySheetPixmapValue &other) const  { return compare(other) <  0; }

    // Check where a pixmap comes from
    enum PixmapSource { LanguageResourcePixmap , ResourcePixmap, FilePixmap };
    static PixmapSource getPixmapSource(QDesignerFormEditorInterface *core, const QString & path);

    PixmapSource pixmapSource(QDesignerFormEditorInterface *core) const { return getPixmapSource(core, m_path); }

    QString path() const;
    void setPath(const QString &path); // passing the empty path resets the pixmap

    int compare(const PropertySheetPixmapValue &other) const;

private:
    QString m_path;
};

// -------------- IconValue: Returned by the property sheet for icons

class PropertySheetIconValueData;

class QDESIGNER_SHARED_EXPORT PropertySheetIconValue
{
 public:
    PropertySheetIconValue(const PropertySheetPixmapValue &pixmap);
    PropertySheetIconValue();
    ~PropertySheetIconValue();
    PropertySheetIconValue(const PropertySheetIconValue &);
    PropertySheetIconValue &operator=(const PropertySheetIconValue &);

    bool operator==(const PropertySheetIconValue &other) const { return equals(other); }
    bool operator!=(const PropertySheetIconValue &other) const { return !equals(other); }
    bool operator<(const PropertySheetIconValue &other) const;

    bool isEmpty() const;

    QString theme() const;
    void setTheme(const QString &);

    PropertySheetPixmapValue pixmap(QIcon::Mode mode, QIcon::State state) const;
    void setPixmap(QIcon::Mode mode, QIcon::State state, const PropertySheetPixmapValue &path); // passing the empty path resets the pixmap

    uint mask() const;
    uint compare(const PropertySheetIconValue &other) const;
    void assign(const PropertySheetIconValue &other, uint mask);

    // Convenience accessors to get themed/unthemed icons.
    PropertySheetIconValue themed() const;
    PropertySheetIconValue unthemed() const;

    typedef QPair<QIcon::Mode, QIcon::State> ModeStateKey;
    typedef QMap<ModeStateKey, PropertySheetPixmapValue> ModeStateToPixmapMap;

    const ModeStateToPixmapMap &paths() const;

private:
    bool equals(const PropertySheetIconValue &rhs) const;
    QSharedDataPointer<PropertySheetIconValueData> m_data;
};

QDESIGNER_SHARED_EXPORT QDebug operator<<(QDebug, const PropertySheetIconValue &);

class QDESIGNER_SHARED_EXPORT DesignerPixmapCache : public QObject
{
    Q_OBJECT
public:
    DesignerPixmapCache(QObject *parent = 0);
    QPixmap pixmap(const PropertySheetPixmapValue &value) const;
    void clear();
signals:
    void reloaded();
private:
    mutable QMap<PropertySheetPixmapValue, QPixmap> m_cache;
    friend class FormWindowBase;
};

class QDESIGNER_SHARED_EXPORT DesignerIconCache : public QObject
{
    Q_OBJECT
public:
    explicit DesignerIconCache(DesignerPixmapCache *pixmapCache, QObject *parent = 0);
    QIcon icon(const PropertySheetIconValue &value) const;
    void clear();
signals:
    void reloaded();
private:
    mutable QMap<PropertySheetIconValue, QIcon> m_cache;
    DesignerPixmapCache *m_pixmapCache;
    friend class FormWindowBase;
};

// -------------- PropertySheetTranslatableData: Base class for translatable properties.
class QDESIGNER_SHARED_EXPORT PropertySheetTranslatableData
{
protected:
    PropertySheetTranslatableData(bool translatable = true,
                                  const QString &disambiguation = QString(),
                                  const QString &comment = QString());
    bool equals(const PropertySheetTranslatableData &rhs) const;

public:
    bool translatable() const                { return m_translatable; }
    void setTranslatable(bool translatable)  { m_translatable = translatable; }
    QString disambiguation() const           { return m_disambiguation; }
    void setDisambiguation(const QString &d) { m_disambiguation = d; }
    QString comment() const                  { return m_comment; }
    void setComment(const QString &comment)  { m_comment = comment; }

private:
    bool m_translatable;
    QString m_disambiguation;
    QString m_comment;
};

// -------------- StringValue: Returned by the property sheet for strings
class QDESIGNER_SHARED_EXPORT PropertySheetStringValue : public PropertySheetTranslatableData
{
public:
    PropertySheetStringValue(const QString &value = QString(), bool translatable = true,
                             const QString &disambiguation = QString(), const QString &comment = QString());

    bool operator==(const PropertySheetStringValue &other) const { return equals(other); }
    bool operator!=(const PropertySheetStringValue &other) const { return !equals(other); }

    QString value() const;
    void setValue(const QString &value);

private:
    bool equals(const PropertySheetStringValue &rhs) const;

    QString m_value;
};

// -------------- StringValue: Returned by the property sheet for string lists
class QDESIGNER_SHARED_EXPORT PropertySheetStringListValue : public PropertySheetTranslatableData
{
public:
    PropertySheetStringListValue(const QStringList &value = QStringList(),
                                 bool translatable = true,
                                 const QString &disambiguation = QString(),
                                 const QString &comment = QString());

    bool operator==(const PropertySheetStringListValue &other) const { return equals(other); }
    bool operator!=(const PropertySheetStringListValue &other) const { return !equals(other); }

    QStringList value() const;
    void setValue(const QStringList &value);

private:
    bool equals(const PropertySheetStringListValue &rhs) const;

    QStringList m_value;
};

// -------------- StringValue: Returned by the property sheet for strings
class QDESIGNER_SHARED_EXPORT PropertySheetKeySequenceValue : public PropertySheetTranslatableData
{
public:
    PropertySheetKeySequenceValue(const QKeySequence &value = QKeySequence(),
                                  bool translatable = true,
                                  const QString &disambiguation = QString(),
                                  const QString &comment = QString());
    PropertySheetKeySequenceValue(const QKeySequence::StandardKey &standardKey,
                                  bool translatable = true,
                                  const QString &disambiguation = QString(),
                                  const QString &comment = QString());

    bool operator==(const PropertySheetKeySequenceValue &other) const { return equals(other); }
    bool operator!=(const PropertySheetKeySequenceValue &other) const { return !equals(other); }

    QKeySequence value() const;
    void setValue(const QKeySequence &value);
    QKeySequence::StandardKey standardKey() const;
    void setStandardKey(const QKeySequence::StandardKey &standardKey);
    bool isStandardKey() const;

private:
    bool equals(const PropertySheetKeySequenceValue &rhs) const;

    QKeySequence m_value;
    QKeySequence::StandardKey m_standardKey;
};

} // namespace qdesigner_internal

QT_END_NAMESPACE


// NOTE: Do not move this code, needed for GCC 3.3
Q_DECLARE_METATYPE(qdesigner_internal::PropertySheetEnumValue)
Q_DECLARE_METATYPE(qdesigner_internal::PropertySheetFlagValue)
Q_DECLARE_METATYPE(qdesigner_internal::PropertySheetPixmapValue)
Q_DECLARE_METATYPE(qdesigner_internal::PropertySheetIconValue)
Q_DECLARE_METATYPE(qdesigner_internal::PropertySheetStringValue)
Q_DECLARE_METATYPE(qdesigner_internal::PropertySheetStringListValue)
Q_DECLARE_METATYPE(qdesigner_internal::PropertySheetKeySequenceValue)


QT_BEGIN_NAMESPACE

namespace qdesigner_internal {


// Create a command to change a text property (that is, create a reset property command if the text is empty)
QDESIGNER_SHARED_EXPORT QDesignerFormWindowCommand *createTextPropertyCommand(const QString &propertyName, const QString &text, QObject *object, QDesignerFormWindowInterface *fw);

// Returns preferred task menu action for managed widget
QDESIGNER_SHARED_EXPORT QAction *preferredEditAction(QDesignerFormEditorInterface *core, QWidget *managedWidget);

// Convenience to run UIC
QDESIGNER_SHARED_EXPORT bool runUIC(const QString &fileName, QByteArray& ba, QString &errorMessage);

// Find a suitable variable name for a class.
QDESIGNER_SHARED_EXPORT QString qtify(const QString &name);

/* UpdateBlocker: Blocks the updates of the widget passed on while in scope.
 * Does nothing if the incoming widget already has updatesEnabled==false
 * which is important to avoid side-effects when putting it into QStackedLayout. */

class QDESIGNER_SHARED_EXPORT UpdateBlocker {
    Q_DISABLE_COPY(UpdateBlocker)

public:
    UpdateBlocker(QWidget *w);
    ~UpdateBlocker();

private:
    QWidget *m_widget;
    const bool m_enabled;
};

namespace Utils {

inline int valueOf(const QVariant &value, bool *ok = 0)
{
    if (value.canConvert<PropertySheetEnumValue>()) {
        if (ok)
            *ok = true;
        return qvariant_cast<PropertySheetEnumValue>(value).value;
    }
    else if (value.canConvert<PropertySheetFlagValue>()) {
        if (ok)
            *ok = true;
        return qvariant_cast<PropertySheetFlagValue>(value).value;
    }
    return value.toInt(ok);
}

inline bool isObjectAncestorOf(QObject *ancestor, QObject *child)
{
    QObject *obj = child;
    while (obj != 0) {
        if (obj == ancestor)
            return true;
        obj = obj->parent();
    }
    return false;
}

inline bool isCentralWidget(QDesignerFormWindowInterface *fw, QWidget *widget)
{
    if (! fw || ! widget)
        return false;

    if (widget == fw->mainContainer())
        return true;

    // ### generalize for other containers
    if (QMainWindow *mw = qobject_cast<QMainWindow*>(fw->mainContainer())) {
        return mw->centralWidget() == widget;
    }

    return false;
}

} // namespace Utils

} // namespace qdesigner_internal

QT_END_NAMESPACE

#endif // QDESIGNER_UTILS_H
