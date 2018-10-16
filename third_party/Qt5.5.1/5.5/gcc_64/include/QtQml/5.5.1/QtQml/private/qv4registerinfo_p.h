/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the QtQml module of the Qt Toolkit.
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

#ifndef QV4REGISTERINFO_P_H
#define QV4REGISTERINFO_P_H

#include <QtCore/QString>

QT_BEGIN_NAMESPACE

namespace QV4 {
namespace JIT {

class RegisterInfo
{
public:
    enum { InvalidRegister = (1 << 29) - 1 };
    enum SavedBy { CallerSaved, CalleeSaved };
    enum RegisterType { RegularRegister, FloatingPointRegister };
    enum Usage { Predefined, RegAlloc };

public:
    RegisterInfo()
        : _savedBy(CallerSaved)
        , _usage(Predefined)
        , _type(RegularRegister)
        , _reg(InvalidRegister)
    {}

    RegisterInfo(int reg, const QString &prettyName, RegisterType type, SavedBy savedBy, Usage usage)
        : _prettyName(prettyName)
        , _savedBy(savedBy)
        , _usage(usage)
        , _type(type)
        , _reg(reg)
    {}

    bool operator==(const RegisterInfo &other) const
    { return _type == other._type && _reg == other._reg; }

    bool isValid() const { return _reg != InvalidRegister; }
    template <typename T> T reg() const { return static_cast<T>(_reg); }
    QString prettyName() const { return _prettyName; }
    bool isCallerSaved() const { return _savedBy == CallerSaved; }
    bool isCalleeSaved() const { return _savedBy == CalleeSaved; }
    bool isFloatingPoint() const { return _type == FloatingPointRegister; }
    bool isRegularRegister() const { return _type == RegularRegister; }
    bool useForRegAlloc() const { return _usage == RegAlloc; }
    bool isPredefined() const { return _usage == Predefined; }

private:
    QString _prettyName;
    unsigned _savedBy : 1;
    unsigned _usage   : 1;
    unsigned _type    : 1;
    unsigned _reg     : 29;
};
typedef QVector<RegisterInfo> RegisterInformation;

} // JIT namespace
} // QV4 namespace

QT_END_NAMESPACE

#endif // QV4REGISTERINFO_P_H
