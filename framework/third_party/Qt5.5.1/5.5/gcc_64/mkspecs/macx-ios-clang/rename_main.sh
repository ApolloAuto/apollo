#!/bin/bash

#############################################################################
##
## Copyright (C) 2015 The Qt Company Ltd.
## Contact: http://www.qt.io/licensing/
##
## This file is the build configuration utility of the Qt Toolkit.
##
## $QT_BEGIN_LICENSE:LGPL21$
## Commercial License Usage
## Licensees holding valid commercial Qt licenses may use this file in
## accordance with the commercial license agreement provided with the
## Software or, alternatively, in accordance with the terms contained in
## a written agreement between you and The Qt Company. For licensing terms
## and conditions see http://www.qt.io/terms-conditions. For further
## information use the contact form at http://www.qt.io/contact-us.
##
## GNU Lesser General Public License Usage
## Alternatively, this file may be used under the terms of the GNU Lesser
## General Public License version 2.1 or version 3 as published by the Free
## Software Foundation and appearing in the file LICENSE.LGPLv21 and
## LICENSE.LGPLv3 included in the packaging of this file. Please review the
## following information to ensure the GNU Lesser General Public License
## requirements will be met: https://www.gnu.org/licenses/lgpl.html and
## http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html.
##
## As a special exception, The Qt Company gives you certain additional
## rights. These rights are described in The Qt Company LGPL Exception
## version 1.1, included in the file LGPL_EXCEPTION.txt in this package.
##
## $QT_END_LICENSE$
##
#############################################################################

if [ $# -ne 2 ]; then
    echo "$0: wrong number of arguments for internal tool used by iOS mkspec"
else
    arch_paths=""
    for a in $2; do
        arch_paths="$arch_paths
$1/$a"
    done
    for f in $(IFS="
"; find $arch_paths -name '*.o'); do
        # Skip object files without the _main symbol
        nm $f 2>/dev/null | grep -q 'T _main$' || continue

        fname=${f#$1/}

        file -b $f | grep -qi 'llvm bit-code' && \
            (cat \
<<EOF >&2
$f:: error: The file '$fname' contains LLVM bitcode, not object code. Automatic main() redirection could not be applied.
note: This is most likely due to the use of link-time optimization (-flto). Please disable LTO, or work around the \
issue by manually renaming your main() function to qtmn():

#ifdef Q_OS_IOS
extern "C" int qtmn(int argc, char *argv[])
#else
int main(int argc, char *argv[])
#endif
EOF
            ) && exit 1

        echo "Found main() in $fname"

        strings -t d - $f | grep '_main\(\.eh\)\?$' | while read match; do
            offset=$(echo $match | cut -d ' ' -f 1)
            symbol=$(echo $match | cut -d ' ' -f 2)

            echo "  Renaming '$symbol' at offset $offset to '${symbol/main/qtmn}'"

            # In-place rename the string (keeping the same length)
            printf '_qtmn' | dd of=$f bs=1 seek=$offset conv=notrunc >/dev/null 2>&1
        done
    done
fi