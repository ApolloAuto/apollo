/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the test suite of the Qt Toolkit.
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

import QtQuick 2.0
import QtTest 1.1
import "testlogger.js" as TestLogger
import Qt.test.qtestroot 1.0

/*!
    \qmltype TestCase
    \inqmlmodule QtTest
    \brief Represents a unit test case
    \since 4.8
    \ingroup qtquicktest

    \section1 Introduction to QML test cases

    Test cases are written as JavaScript functions within a TestCase
    type:

    \code
    import QtQuick 2.0
    import QtTest 1.0

    TestCase {
        name: "MathTests"

        function test_math() {
            compare(2 + 2, 4, "2 + 2 = 4")
        }

        function test_fail() {
            compare(2 + 2, 5, "2 + 2 = 5")
        }
    }
    \endcode

    Functions whose names start with "test_" are treated as test cases
    to be executed.  The \l name property is used to prefix the functions
    in the output:

    \code
    ********* Start testing of MathTests *********
    Config: Using QTest library 4.7.2, Qt 4.7.2
    PASS   : MathTests::initTestCase()
    FAIL!  : MathTests::test_fail() 2 + 2 = 5
       Actual (): 4
       Expected (): 5
       Loc: [/home/.../tst_math.qml(12)]
    PASS   : MathTests::test_math()
    PASS   : MathTests::cleanupTestCase()
    Totals: 3 passed, 1 failed, 0 skipped
    ********* Finished testing of MathTests *********
    \endcode

    Because of the way JavaScript properties work, the order in which the
    test functions are found is unpredictable.  To assist with predictability,
    the test framework will sort the functions on ascending order of name.
    This can help when there are two tests that must be run in order.

    Multiple TestCase types can be supplied.  The test program will exit
    once they have all completed.  If a test case doesn't need to run
    (because a precondition has failed), then \l optional can be set to true.

    \section1 Data-driven tests

    Table data can be provided to a test using a function name that ends
    with "_data". Alternatively, the \c init_data() function can be used
    to provide default test data for all test functions in a TestCase type:


    \code
    import QtQuick 2.0
    import QtTest 1.1

    TestCase {
        name: "DataTests"

        function init_data() {
          return [
               {tag:"init_data_1", a:1, b:2, answer: 3},
               {tag:"init_data_2", a:2, b:4, answer: 6}
          ];
        }

        function test_table_data() {
            return [
                {tag: "2 + 2 = 4", a: 2, b: 2, answer: 4 },
                {tag: "2 + 6 = 8", a: 2, b: 6, answer: 8 },
            ]
        }

        function test_table(data) {
            //data comes from test_table_data
            compare(data.a + data.b, data.answer)
        }

        function test__default_table(data) {
            //data comes from init_data
            compare(data.a + data.b, data.answer)
        }
    }
    \endcode

    The test framework will iterate over all of the rows in the table
    and pass each row to the test function.  As shown, the columns can be
    extracted for use in the test.  The \c tag column is special - it is
    printed by the test framework when a row fails, to help the reader
    identify which case failed amongst a set of otherwise passing tests.

    \section1 Benchmarks

    Functions whose names start with "benchmark_" will be run multiple
    times with the Qt benchmark framework, with an average timing value
    reported for the runs.  This is equivalent to using the \c{QBENCHMARK}
    macro in the C++ version of QTestLib.

    \code
    TestCase {
        id: top
        name: "CreateBenchmark"

        function benchmark_create_component() {
            var component = Qt.createComponent("item.qml")
            var obj = component.createObject(top)
            obj.destroy()
            component.destroy()
        }
    }

    RESULT : CreateBenchmark::benchmark_create_component:
         0.23 msecs per iteration (total: 60, iterations: 256)
    PASS   : CreateBenchmark::benchmark_create_component()
    \endcode

    To get the effect of the \c{QBENCHMARK_ONCE} macro, prefix the test
    function name with "benchmark_once_".

    \section1 Simulating keyboard and mouse events

    The keyPress(), keyRelease(), and keyClick() methods can be used
    to simulate keyboard events within unit tests.  The events are
    delivered to the currently focused QML item. You can pass either
    a Qt.Key enum value or a latin1 char (string of length one)

    \code
    Rectangle {
        width: 50; height: 50
        focus: true

        TestCase {
            name: "KeyClick"
            when: windowShown

            function test_key_click() {
                keyClick(Qt.Key_Left)
                keyClick("a")
                ...
            }
        }
    }
    \endcode

    The mousePress(), mouseRelease(), mouseClick(), mouseDoubleClick(), mouseDoubleClickSequence()
    and mouseMove() methods can be used to simulate mouse events in a
    similar fashion.

    \b{Note:} keyboard and mouse events can only be delivered once the
    main window has been shown.  Attempts to deliver events before then
    will fail.  Use the \l when and windowShown properties to track
    when the main window has been shown.

    \sa {QtTest::SignalSpy}{SignalSpy}, {Qt Quick Test Reference Documentation}
*/


Item {
    id: testCase
    visible: false
    TestUtil {
        id:util
    }

    /*!
        \qmlproperty string TestCase::name

        This property defines the name of the test case for result reporting.
        The default is the empty string.

        \code
        TestCase {
            name: "ButtonTests"
            ...
        }
        \endcode
    */
    property string name

    /*!
        \qmlproperty bool TestCase::when

        This property should be set to true when the application wants
        the test cases to run.  The default value is true.  In the following
        example, a test is run when the user presses the mouse button:

        \code
        Rectangle {
            id: foo
            width: 640; height: 480
            color: "cyan"

            MouseArea {
                id: area
                anchors.fill: parent
            }

            property bool bar: true

            TestCase {
                name: "ItemTests"
                when: area.pressed
                id: test1

                function test_bar() {
                    verify(bar)
                }
            }
        }
        \endcode

        The test application will exit once all \l TestCase types
        have been triggered and have run.  The \l optional property can
        be used to exclude a \l TestCase type.

        \sa optional, completed
    */
    property bool when: true

    /*!
        \qmlproperty bool TestCase::completed

        This property will be set to true once the test case has completed
        execution.  Test cases are only executed once.  The initial value
        is false.

        \sa running, when
    */
    property bool completed: false

    /*!
        \qmlproperty bool TestCase::running

        This property will be set to true while the test case is running.
        The initial value is false, and the value will become false again
        once the test case completes.

        \sa completed, when
    */
    property bool running: false

    /*!
        \qmlproperty bool TestCase::optional

        Multiple \l TestCase types can be supplied in a test application.
        The application will exit once they have all completed.  If a test case
        does not need to run (because a precondition has failed), then this
        property can be set to true.  The default value is false.

        \code
        TestCase {
            when: false
            optional: true
            function test_not_run() {
                verify(false)
            }
        }
        \endcode

        \sa when, completed
    */
    property bool optional: false

    /*!
        \qmlproperty bool TestCase::windowShown

        This property will be set to true after the QML viewing window has
        been displayed.  Normally test cases run as soon as the test application
        is loaded and before a window is displayed.  If the test case involves
        visual types and behaviors, then it may need to be delayed until
        after the window is shown.

        \code
        Button {
            id: button
            onClicked: text = "Clicked"
            TestCase {
                name: "ClickTest"
                when: windowShown
                function test_click() {
                    button.clicked();
                    compare(button.text, "Clicked");
                }
            }
        }
        \endcode
    */
    property bool windowShown: QTestRootObject.windowShown

    // Internal private state.  Identifiers prefixed with qtest are reserved.
    /*! \internal */
    property bool qtest_prevWhen: true
    /*! \internal */
    property int qtest_testId: -1
    /*! \internal */
    property bool qtest_componentCompleted : false
    /*! \internal */
    property var qtest_testCaseResult
    /*! \internal */
    property var qtest_results: qtest_results_normal
    /*! \internal */
    TestResult { id: qtest_results_normal }
    /*! \internal */
    property var qtest_events: qtest_events_normal
    TestEvent { id: qtest_events_normal }

    /*!
        \qmlmethod TestCase::fail(message = "")

        Fails the current test case, with the optional \a message.
        Similar to \c{QFAIL(message)} in C++.
    */
    function fail(msg) {
        if (msg === undefined)
            msg = "";
        qtest_results.fail(msg, util.callerFile(), util.callerLine())
        throw new Error("QtQuickTest::fail")
    }

    /*! \internal */
    function qtest_fail(msg, frame) {
        if (msg === undefined)
            msg = "";
        qtest_results.fail(msg, util.callerFile(frame), util.callerLine(frame))
        throw new Error("QtQuickTest::fail")
    }

    /*!
        \qmlmethod TestCase::verify(condition, message = "")

        Fails the current test case if \a condition is false, and
        displays the optional \a message.  Similar to \c{QVERIFY(condition)}
        or \c{QVERIFY2(condition, message)} in C++.
    */
    function verify(cond, msg) {
        if (msg === undefined)
            msg = "";
        if (!qtest_results.verify(cond, msg, util.callerFile(), util.callerLine()))
            throw new Error("QtQuickTest::fail")
    }

    /*! \internal */
    // Determine what is o.
    // Discussions and reference: http://philrathe.com/articles/equiv
    // Test suites: http://philrathe.com/tests/equiv
    // Author: Philippe Rathé <prathe@gmail.com>
    function qtest_typeof(o) {
        if (typeof o === "undefined") {
            return "undefined";

        // consider: typeof null === object
        } else if (o === null) {
            return "null";

        } else if (o.constructor === String) {
            return "string";

        } else if (o.constructor === Boolean) {
            return "boolean";

        } else if (o.constructor === Number) {

            if (isNaN(o)) {
                return "nan";
            } else {
                return "number";
            }
        // consider: typeof [] === object
        } else if (o instanceof Array) {
            return "array";

        // consider: typeof new Date() === object
        } else if (o instanceof Date) {
            return "date";

        // consider: /./ instanceof Object;
        //           /./ instanceof RegExp;
        //          typeof /./ === "function"; // => false in IE and Opera,
        //                                          true in FF and Safari
        } else if (o instanceof RegExp) {
            return "regexp";

        } else if (typeof o === "object") {
            if ("mapFromItem" in o && "mapToItem" in o) {
                return "declarativeitem";  // @todo improve detection of declarative items
            } else if ("x" in o && "y" in o && "z" in o) {
                return "vector3d"; // Qt 3D vector
            }
            return "object";
        } else if (o instanceof Function) {
            return "function";
        } else {
            return undefined;
        }
    }

    /*! \internal */
    // Test for equality
    // Large parts contain sources from QUnit or http://philrathe.com
    // Discussions and reference: http://philrathe.com/articles/equiv
    // Test suites: http://philrathe.com/tests/equiv
    // Author: Philippe Rathé <prathe@gmail.com>
    function qtest_compareInternal(act, exp) {
        var success = false;
        if (act === exp) {
            success = true; // catch the most you can
        } else if (act === null || exp === null || typeof act === "undefined" || typeof exp === "undefined") {
            success = false; // don't lose time with error prone cases
        } else {
            var typeExp = qtest_typeof(exp), typeAct = qtest_typeof(act)
            if (typeExp !== typeAct) {
                // allow object vs string comparison (e.g. for colors)
                // else break on different types
                if ((typeExp === "string" && (typeAct === "object") || typeAct == "declarativeitem")
                 || ((typeExp === "object" || typeExp == "declarativeitem") && typeAct === "string")) {
                    success = (act == exp)
                }
            } else if (typeExp === "string" || typeExp === "boolean" ||
                       typeExp === "null" || typeExp === "undefined") {
                if (exp instanceof act.constructor || act instanceof exp.constructor) {
                    // to catch short annotaion VS 'new' annotation of act declaration
                    // e.g. var i = 1;
                    //      var j = new Number(1);
                    success = (act == exp)
                } else {
                    success = (act === exp)
                }
            } else if (typeExp === "nan") {
                success = isNaN(act);
            } else if (typeExp === "number") {
                // Use act fuzzy compare if the two values are floats
                if (Math.abs(act - exp) <= 0.00001) {
                    success = true
                }
            } else if (typeExp === "array") {
                success = qtest_compareInternalArrays(act, exp)
            } else if (typeExp === "object") {
                success = qtest_compareInternalObjects(act, exp)
            } else if (typeExp === "declarativeitem") {
                success = qtest_compareInternalObjects(act, exp) // @todo improve comparison of declarative items
            } else if (typeExp === "vector3d") {
                success = (Math.abs(act.x - exp.x) <= 0.00001 &&
                           Math.abs(act.y - exp.y) <= 0.00001 &&
                           Math.abs(act.z - exp.z) <= 0.00001)
            } else if (typeExp === "date") {
                success = (act.valueOf() === exp.valueOf())
            } else if (typeExp === "regexp") {
                success = (act.source === exp.source && // the regex itself
                           act.global === exp.global && // and its modifers (gmi) ...
                           act.ignoreCase === exp.ignoreCase &&
                           act.multiline === exp.multiline)
            }
        }
        return success
    }

    /*! \internal */
    function qtest_compareInternalObjects(act, exp) {
        var i;
        var eq = true; // unless we can proove it
        var aProperties = [], bProperties = []; // collection of strings

        // comparing constructors is more strict than using instanceof
        if (act.constructor !== exp.constructor) {
            return false;
        }

        for (i in act) { // be strict: don't ensures hasOwnProperty and go deep
            aProperties.push(i); // collect act's properties
            if (!qtest_compareInternal(act[i], exp[i])) {
                eq = false;
                break;
            }
        }

        for (i in exp) {
            bProperties.push(i); // collect exp's properties
        }

        // Ensures identical properties name
        return eq && qtest_compareInternal(aProperties.sort(), bProperties.sort());

    }

    /*! \internal */
    function qtest_compareInternalArrays(actual, expected) {
        if (actual.length != expected.length) {
            return false
        }

        for (var i = 0, len = actual.length; i < len; i++) {
            if (!qtest_compareInternal(actual[i], expected[i])) {
                return false
            }
        }

        return true
    }

    /*!
        \qmlmethod TestCase::compare(actual, expected, message = "")

        Fails the current test case if \a actual is not the same as
        \a expected, and displays the optional \a message.  Similar
        to \c{QCOMPARE(actual, expected)} in C++.

        \sa tryCompare(), fuzzyCompare
    */
    function compare(actual, expected, msg) {
        var act = qtest_results.stringify(actual)
        var exp = qtest_results.stringify(expected)

        var success = qtest_compareInternal(actual, expected)
        if (msg === undefined) {
            if (success)
                msg = "COMPARE()"
            else
                msg = "Compared values are not the same"
        }
        if (!qtest_results.compare(success, msg, act, exp, util.callerFile(), util.callerLine())) {
            throw new Error("QtQuickTest::fail")
        }
    }

    /*!
        \qmlmethod TestCase::fuzzyCompare(actual, expected, delta, message = "")

        Fails the current test case if the difference betwen \a actual and \a expected
        is greater than \a delta, and displays the optional \a message.  Similar
        to \c{qFuzzyCompare(actual, expected)} in C++ but with a required \a delta value.

        This function can also be used for color comparisons if both the \a actual and
        \a expected values can be converted into color values. If any of the differences
        for RGBA channel values are greater than \a delta, the test fails.

        \sa tryCompare(), compare()
    */
    function fuzzyCompare(actual, expected, delta, msg) {
        if (delta === undefined)
            qtest_fail("A delta value is required for fuzzyCompare", 2)

        var success = qtest_results.fuzzyCompare(actual, expected, delta)
        if (msg === undefined) {
            if (success)
                msg = "FUZZYCOMPARE()"
            else
                msg = "Compared values are not the same with delta(" + delta + ")"
        }

        if (!qtest_results.compare(success, msg, actual, expected, util.callerFile(), util.callerLine())) {
            throw new Error("QtQuickTest::fail")
        }
    }

    /*!
        \qmlmethod object TestCase::grabImage(item)

        Returns a snapshot image object of the given \a item.

        The returned image object has the following methods:
        \list
        \li red(x, y) Returns the red channel value of the pixel at \a x, \a y position
        \li green(x, y) Returns the green channel value of the pixel at \a x, \a y position
        \li blue(x, y) Returns the blue channel value of the pixel at \a x, \a y position
        \li alpha(x, y) Returns the alpha channel value of the pixel at \a x, \a y position
        \li pixel(x, y) Returns the color value of the pixel at \a x, \a y position
        For example:

        \code
        var image = grabImage(rect);
        compare(image.red(10, 10), 255);
        compare(image.pixel(20, 20), Qt.rgba(255, 0, 0, 255));
        \endcode

        \endlist

        \sa
    */
    function grabImage(item) {
        return qtest_results.grabImage(item);
    }

    /*!
        \since 5.4
        \qmlmethod QtObject TestCase::findChild(parent, objectName)

        Returns the first child of \a parent with \a objectName, or \c null if
        no such item exists. Both visual and non-visual children are searched
        recursively, with visual children being searched first.

        \code
        compare(findChild(item, "childObject"), expectedChildObject);
        \endcode
    */
    function findChild(parent, objectName) {
        // First, search the visual item hierarchy.
        var child = qtest_findVisualChild(parent, objectName);
        if (child)
            return child;

        // If it's not a visual child, it might be a QObject child.
        return qtest_results.findChild(parent, objectName);
    }

    /*! \internal */
    function qtest_findVisualChild(parent, objectName) {
        if (!parent || parent.children === undefined)
            return null;

        for (var i = 0; i < parent.children.length; ++i) {
            // Is this direct child of ours the child we're after?
            var child = parent.children[i];
            if (child.objectName === objectName)
                return child;
        }

        for (i = 0; i < parent.children.length; ++i) {
            // Try the direct child's children.
            child = qtest_findVisualChild(parent.children[i], objectName);
            if (child)
                return child;
        }
        return null;
    }

    /*!
        \qmlmethod TestCase::tryCompare(obj, property, expected, timeout = 5000, message = "")

        Fails the current test case if the specified \a property on \a obj
        is not the same as \a expected, and displays the optional \a message.
        The test will be retried multiple times until the
        \a timeout (in milliseconds) is reached.

        This function is intended for testing applications where a property
        changes value based on asynchronous events.  Use compare() for testing
        synchronous property changes.

        \code
        tryCompare(img, "status", BorderImage.Ready)
        compare(img.width, 120)
        compare(img.height, 120)
        compare(img.horizontalTileMode, BorderImage.Stretch)
        compare(img.verticalTileMode, BorderImage.Stretch)
        \endcode

        SignalSpy::wait() provides an alternative method to wait for a
        signal to be emitted.

        \sa compare(), SignalSpy::wait()
    */
    function tryCompare(obj, prop, value, timeout, msg) {
        if (arguments.length == 2) {
            qtest_results.fail("A value is required for tryCompare",
                        util.callerFile(), util.callerLine())
            throw new Error("QtQuickTest::fail")
        }
        if (timeout !== undefined && typeof(timeout) != "number") {
            qtest_results.fail("timeout should be a number",
                        util.callerFile(), util.callerLine())
            throw new Error("QtQuickTest::fail")
        }
        if (!timeout)
            timeout = 5000
        if (msg === undefined)
            msg = "property " + prop
        if (!qtest_compareInternal(obj[prop], value))
            wait(0)
        var i = 0
        while (i < timeout && !qtest_compareInternal(obj[prop], value)) {
            wait(50)
            i += 50
        }
        var actual = obj[prop]
        var act = qtest_results.stringify(actual)
        var exp = qtest_results.stringify(value)
        var success = qtest_compareInternal(actual, value)
        if (!qtest_results.compare(success, msg, act, exp, util.callerFile(), util.callerLine()))
            throw new Error("QtQuickTest::fail")
    }

    /*!
        \qmlmethod TestCase::skip(message = "")

        Skips the current test case and prints the optional \a message.
        If this is a data-driven test, then only the current row is skipped.
        Similar to \c{QSKIP(message)} in C++.
    */
    function skip(msg) {
        if (msg === undefined)
            msg = ""
        qtest_results.skip(msg, util.callerFile(), util.callerLine())
        throw new Error("QtQuickTest::skip")
    }

    /*!
        \qmlmethod TestCase::expectFail(tag, message)

        In a data-driven test, marks the row associated with \a tag as
        expected to fail.  When the fail occurs, display the \a message,
        abort the test, and mark the test as passing.  Similar to
        \c{QEXPECT_FAIL(tag, message, Abort)} in C++.

        If the test is not data-driven, then \a tag must be set to
        the empty string.

        \sa expectFailContinue()
    */
    function expectFail(tag, msg) {
        if (tag === undefined) {
            warn("tag argument missing from expectFail()")
            tag = ""
        }
        if (msg === undefined) {
            warn("message argument missing from expectFail()")
            msg = ""
        }
        if (!qtest_results.expectFail(tag, msg, util.callerFile(), util.callerLine()))
            throw new Error("QtQuickTest::expectFail")
    }

    /*!
        \qmlmethod TestCase::expectFailContinue(tag, message)

        In a data-driven test, marks the row associated with \a tag as
        expected to fail.  When the fail occurs, display the \a message,
        and then continue the test.  Similar to
        \c{QEXPECT_FAIL(tag, message, Continue)} in C++.

        If the test is not data-driven, then \a tag must be set to
        the empty string.

        \sa expectFail()
    */
    function expectFailContinue(tag, msg) {
        if (tag === undefined) {
            warn("tag argument missing from expectFailContinue()")
            tag = ""
        }
        if (msg === undefined) {
            warn("message argument missing from expectFailContinue()")
            msg = ""
        }
        if (!qtest_results.expectFailContinue(tag, msg, util.callerFile(), util.callerLine()))
            throw new Error("QtQuickTest::expectFail")
    }

    /*!
        \qmlmethod TestCase::warn(message)

        Prints \a message as a warning message.  Similar to
        \c{QWARN(message)} in C++.

        \sa ignoreWarning()
    */
    function warn(msg) {
        if (msg === undefined)
            msg = ""
        qtest_results.warn(msg, util.callerFile(), util.callerLine());
    }

    /*!
        \qmlmethod TestCase::ignoreWarning(message)

        Marks \a message as an ignored warning message.  When it occurs,
        the warning will not be printed and the test passes.  If the message
        does not occur, then the test will fail.  Similar to
        \c{QTest::ignoreMessage(QtWarningMsg, message)} in C++.

        \sa warn()
    */
    function ignoreWarning(msg) {
        if (msg === undefined)
            msg = ""
        qtest_results.ignoreWarning(msg)
    }

    /*!
        \qmlmethod TestCase::wait(ms)

        Waits for \a ms milliseconds while processing Qt events.

        \sa sleep(), waitForRendering()
    */
    function wait(ms) {
        qtest_results.wait(ms)
    }

    /*!
        \qmlmethod TestCase::waitForRendering(item, timeout = 5000)

        Waits for \a timeout milliseconds or until the \a item is rendered by the renderer.
        Returns true if \c item is rendered in \a timeout milliseconds, otherwise returns false.
        The default \a timeout value is 5000.

        \sa sleep(), wait()
    */
    function waitForRendering(item, timeout) {
        if (timeout === undefined)
            timeout = 5000
        if (!item)
            qtest_fail("No item given to waitForRendering", 1)
        return qtest_results.waitForRendering(item, timeout)
    }

    /*!
        \qmlmethod TestCase::sleep(ms)

        Sleeps for \a ms milliseconds without processing Qt events.

        \sa wait(), waitForRendering()
    */
    function sleep(ms) {
        qtest_results.sleep(ms)
    }

    /*!
        \qmlmethod TestCase::keyPress(key, modifiers = Qt.NoModifier, delay = -1)

        Simulates pressing a \a key with an optional \a modifier on the currently
        focused item.  If \a delay is larger than 0, the test will wait for
        \a delay milliseconds.

        \b{Note:} At some point you should release the key using keyRelease().

        \sa keyRelease(), keyClick()
    */
    function keyPress(key, modifiers, delay) {
        if (modifiers === undefined)
            modifiers = Qt.NoModifier
        if (delay == undefined)
            delay = -1
        if (typeof(key) == "string" && key.length == 1) {
            if (!qtest_events.keyPressChar(key, modifiers, delay))
                qtest_fail("window not shown", 2)
        } else {
            if (!qtest_events.keyPress(key, modifiers, delay))
                qtest_fail("window not shown", 2)
        }
    }

    /*!
        \qmlmethod TestCase::keyRelease(key, modifiers = Qt.NoModifier, delay = -1)

        Simulates releasing a \a key with an optional \a modifier on the currently
        focused item.  If \a delay is larger than 0, the test will wait for
        \a delay milliseconds.

        \sa keyPress(), keyClick()
    */
    function keyRelease(key, modifiers, delay) {
        if (modifiers === undefined)
            modifiers = Qt.NoModifier
        if (delay == undefined)
            delay = -1
        if (typeof(key) == "string" && key.length == 1) {
            if (!qtest_events.keyReleaseChar(key, modifiers, delay))
                qtest_fail("window not shown", 2)
        } else {
            if (!qtest_events.keyRelease(key, modifiers, delay))
                qtest_fail("window not shown", 2)
        }
    }

    /*!
        \qmlmethod TestCase::keyClick(key, modifiers = Qt.NoModifier, delay = -1)

        Simulates clicking of \a key with an optional \a modifier on the currently
        focused item.  If \a delay is larger than 0, the test will wait for
        \a delay milliseconds.

        \sa keyPress(), keyRelease()
    */
    function keyClick(key, modifiers, delay) {
        if (modifiers === undefined)
            modifiers = Qt.NoModifier
        if (delay == undefined)
            delay = -1
        if (typeof(key) == "string" && key.length == 1) {
            if (!qtest_events.keyClickChar(key, modifiers, delay))
                qtest_fail("window not shown", 2)
        } else {
            if (!qtest_events.keyClick(key, modifiers, delay))
                qtest_fail("window not shown", 2)
        }
    }

    /*!
        \qmlmethod TestCase::mousePress(item, x = item.width / 2, y = item.height / 2, button = Qt.LeftButton, modifiers = Qt.NoModifier, delay = -1)

        Simulates pressing a mouse \a button with an optional \a modifier
        on an \a item.  The position is defined by \a x and \a y.
        If \a x or \a y are not defined the position will be the center of \a item.
        If \a delay is specified, the test will wait for the specified amount of
        milliseconds before the press.

        The position given by \a x and \a y is transformed from the co-ordinate
        system of \a item into window co-ordinates and then delivered.
        If \a item is obscured by another item, or a child of \a item occupies
        that position, then the event will be delivered to the other item instead.

        \sa mouseRelease(), mouseClick(), mouseDoubleClick(), mouseDoubleClickSequence(), mouseMove(), mouseDrag(), mouseWheel()
    */
    function mousePress(item, x, y, button, modifiers, delay) {
        if (button === undefined)
            button = Qt.LeftButton
        if (modifiers === undefined)
            modifiers = Qt.NoModifier
        if (delay == undefined)
            delay = -1
        if (x === undefined)
            x = item.width / 2
        if (y === undefined)
            y = item.height / 2
        if (!qtest_events.mousePress(item, x, y, button, modifiers, delay))
            qtest_fail("window not shown", 2)
    }

    /*!
        \qmlmethod TestCase::mouseRelease(item, x = item.width / 2, y = item.height / 2, button = Qt.LeftButton, modifiers = Qt.NoModifier, delay = -1)

        Simulates releasing a mouse \a button with an optional \a modifier
        on an \a item.  The position of the release is defined by \a x and \a y.
        If \a x or \a y are not defined the position will be the center of \a item.
        If \a delay is specified, the test will wait for the specified amount of
        milliseconds before releasing the button.

        The position given by \a x and \a y is transformed from the co-ordinate
        system of \a item into window co-ordinates and then delivered.
        If \a item is obscured by another item, or a child of \a item occupies
        that position, then the event will be delivered to the other item instead.

        \sa mousePress(), mouseClick(), mouseDoubleClick(), mouseDoubleClickSequence(), mouseMove(), mouseDrag(), mouseWheel()
    */
    function mouseRelease(item, x, y, button, modifiers, delay) {
        if (button === undefined)
            button = Qt.LeftButton
        if (modifiers === undefined)
            modifiers = Qt.NoModifier
        if (delay == undefined)
            delay = -1
        if (x === undefined)
            x = item.width / 2
        if (y === undefined)
            y = item.height / 2
        if (!qtest_events.mouseRelease(item, x, y, button, modifiers, delay))
            qtest_fail("window not shown", 2)
    }

    /*!
        \qmlmethod TestCase::mouseDrag(item, x, y, dx, dy, button = Qt.LeftButton, modifiers = Qt.NoModifier, delay = -1)

        Simulates dragging the mouse on an \a item with \a button pressed and an optional \a modifier.
        The initial drag position is defined by \a x and \a y,
        and drag distance is defined by \a dx and \a dy. If \a delay is specified,
        the test will wait for the specified amount of milliseconds before releasing the button.

        The position given by \a x and \a y is transformed from the co-ordinate
        system of \a item into window co-ordinates and then delivered.
        If \a item is obscured by another item, or a child of \a item occupies
        that position, then the event will be delivered to the other item instead.

        Note: this method does not imply a drop action, to make a drop, an additional
        mouseRelease(item, x + dx, y + dy) is needed.

        \sa mousePress(), mouseClick(), mouseDoubleClick(), mouseDoubleClickSequence(), mouseMove(), mouseRelease(), mouseWheel()
    */
    function mouseDrag(item, x, y, dx, dy, button, modifiers, delay) {
        if (item.x === undefined || item.y === undefined)
            return
        if (button === undefined)
            button = Qt.LeftButton
        if (modifiers === undefined)
            modifiers = Qt.NoModifier
        if (delay == undefined)
            delay = -1

        // Divide dx and dy to have intermediate mouseMove while dragging
        // Fractions of dx/dy need be superior to the dragThreshold
        // to make the drag works though
        var ddx = Math.round(dx/3)
        if (ddx < (util.dragThreshold + 1))
            ddx = 0
        var ddy = Math.round(dy/3)
        if (ddy < (util.dragThreshold + 1))
            ddy = 0

        mousePress(item, x, y, button, modifiers, delay)
        //trigger dragging
        mouseMove(item, x + util.dragThreshold + 1, y + util.dragThreshold + 1, delay, button)
        if (ddx > 0 || ddy > 0) {
            mouseMove(item, x + ddx, y + ddy, delay, button)
            mouseMove(item, x + 2*ddx, y + 2*ddy, delay, button)
        }
        mouseMove(item, x + dx, y + dy, delay, button)
        mouseRelease(item, x + dx, y + dy, button, modifiers, delay)
    }

    /*!
        \qmlmethod TestCase::mouseClick(item, x = item.width / 2, y = item.height / 2, button = Qt.LeftButton, modifiers = Qt.NoModifier, delay = -1)

        Simulates clicking a mouse \a button with an optional \a modifier
        on an \a item.  The position of the click is defined by \a x and \a y.
        If \a x and \a y are not defined the position will be the center of \a item.
        If \a delay is specified, the test will wait for the specified amount of
        milliseconds before pressing and before releasing the button.

        The position given by \a x and \a y is transformed from the co-ordinate
        system of \a item into window co-ordinates and then delivered.
        If \a item is obscured by another item, or a child of \a item occupies
        that position, then the event will be delivered to the other item instead.

        \sa mousePress(), mouseRelease(), mouseDoubleClick(), mouseDoubleClickSequence(), mouseMove(), mouseDrag(), mouseWheel()
    */
    function mouseClick(item, x, y, button, modifiers, delay) {
        if (button === undefined)
            button = Qt.LeftButton
        if (modifiers === undefined)
            modifiers = Qt.NoModifier
        if (delay == undefined)
            delay = -1
        if (x === undefined)
            x = item.width / 2
        if (y === undefined)
            y = item.height / 2
        if (!qtest_events.mouseClick(item, x, y, button, modifiers, delay))
            qtest_fail("window not shown", 2)
    }

    /*!
        \qmlmethod TestCase::mouseDoubleClick(item, x = item.width / 2, y = item.height / 2, button = Qt.LeftButton, modifiers = Qt.NoModifier, delay = -1)

        Simulates double-clicking a mouse \a button with an optional \a modifier
        on an \a item.  The position of the click is defined by \a x and \a y.
        If \a x and \a y are not defined the position will be the center of \a item.
        If \a delay is specified, the test will wait for the specified amount of
        milliseconds before pressing and before releasing the button.

        The position given by \a x and \a y is transformed from the co-ordinate
        system of \a item into window co-ordinates and then delivered.
        If \a item is obscured by another item, or a child of \a item occupies
        that position, then the event will be delivered to the other item instead.

        \sa mouseDoubleClickSequence(), mousePress(), mouseRelease(), mouseClick(), mouseMove(), mouseDrag(), mouseWheel()
    */
    function mouseDoubleClick(item, x, y, button, modifiers, delay) {
        if (button === undefined)
            button = Qt.LeftButton
        if (modifiers === undefined)
            modifiers = Qt.NoModifier
        if (delay == undefined)
            delay = -1
        if (x === undefined)
            x = item.width / 2
        if (y === undefined)
            y = item.height / 2
        if (!qtest_events.mouseDoubleClick(item, x, y, button, modifiers, delay))
            qtest_fail("window not shown", 2)
    }

    /*!
        \qmlmethod TestCase::mouseDoubleClickSequence(item, x = item.width / 2, y = item.height / 2, button = Qt.LeftButton, modifiers = Qt.NoModifier, delay = -1)

        Simulates the full sequence of events generated by double-clicking a mouse
        \a button with an optional \a modifier on an \a item.

        This method reproduces the sequence of mouse events generated when a user makes
        a double click: Press-Release-Press-DoubleClick-Release.

        The position of the click is defined by \a x and \a y.
        If \a x and \a y are not defined the position will be the center of \a item.
        If \a delay is specified, the test will wait for the specified amount of
        milliseconds before pressing and before releasing the button.

        The position given by \a x and \a y is transformed from the co-ordinate
        system of \a item into window co-ordinates and then delivered.
        If \a item is obscured by another item, or a child of \a item occupies
        that position, then the event will be delivered to the other item instead.

        This QML method was introduced in Qt 5.5.

        \sa mouseDoubleClick(), mousePress(), mouseRelease(), mouseClick(), mouseMove(), mouseDrag(), mouseWheel()
    */
    function mouseDoubleClickSequence(item, x, y, button, modifiers, delay) {
        if (button === undefined)
            button = Qt.LeftButton
        if (modifiers === undefined)
            modifiers = Qt.NoModifier
        if (delay == undefined)
            delay = -1
        if (x === undefined)
            x = item.width / 2
        if (y === undefined)
            y = item.height / 2
        if (!qtest_events.mouseDoubleClickSequence(item, x, y, button, modifiers, delay))
            qtest_fail("window not shown", 2)
    }

    /*!
        \qmlmethod TestCase::mouseMove(item, x, y, delay = -1)

        Moves the mouse pointer to the position given by \a x and \a y within
        \a item.  If a \a delay (in milliseconds) is given, the test will wait
        before moving the mouse pointer.

        The position given by \a x and \a y is transformed from the co-ordinate
        system of \a item into window co-ordinates and then delivered.
        If \a item is obscured by another item, or a child of \a item occupies
        that position, then the event will be delivered to the other item instead.

        \sa mousePress(), mouseRelease(), mouseClick(), mouseDoubleClick(), mouseDoubleClickSequence(), mouseDrag(), mouseWheel()
    */
    function mouseMove(item, x, y, delay, buttons) {
        if (delay == undefined)
            delay = -1
        if (buttons == undefined)
            buttons = Qt.NoButton
        if (!qtest_events.mouseMove(item, x, y, delay, buttons))
            qtest_fail("window not shown", 2)
    }

    /*!
        \qmlmethod TestCase::mouseWheel(item, x, y, xDelta, yDelta, button = Qt.LeftButton, modifiers = Qt.NoModifier, delay = -1)

        Simulates rotating the mouse wheel on an \a item with \a button pressed and an optional \a modifier.
        The position of the wheel event is defined by \a x and \a y.
        If \a delay is specified, the test will wait for the specified amount of milliseconds before releasing the button.

        The position given by \a x and \a y is transformed from the co-ordinate
        system of \a item into window co-ordinates and then delivered.
        If \a item is obscured by another item, or a child of \a item occupies
        that position, then the event will be delivered to the other item instead.

        The \a xDelta and \a yDelta contain the wheel rotation distance in eighths of a degree. see \l QWheelEvent::angleDelta() for more details.

        \sa mousePress(), mouseClick(), mouseDoubleClick(), mouseDoubleClickSequence(), mouseMove(), mouseRelease(), mouseDrag(), QWheelEvent::angleDelta()
    */
    function mouseWheel(item, x, y, xDelta, yDelta, buttons, modifiers, delay) {
        if (delay == undefined)
            delay = -1
        if (buttons == undefined)
            buttons = Qt.NoButton
        if (modifiers === undefined)
            modifiers = Qt.NoModifier
        if (xDelta == undefined)
            xDelta = 0
        if (yDelta == undefined)
            yDelta = 0
        if (!qtest_events.mouseWheel(item, x, y, buttons, modifiers, xDelta, yDelta, delay))
            qtest_fail("window not shown", 2)
   }


    // Functions that can be overridden in subclasses for init/cleanup duties.
    /*!
        \qmlmethod TestCase::initTestCase()

        This function is called before any other test functions in the
        \l TestCase type.  The default implementation does nothing.
        The application can provide its own implementation to perform
        test case initialization.

        \sa cleanupTestCase(), init()
    */
    function initTestCase() {}

    /*!
        \qmlmethod TestCase::cleanupTestCase()

        This function is called after all other test functions in the
        \l TestCase type have completed.  The default implementation
        does nothing.  The application can provide its own implementation
        to perform test case cleanup.

        \sa initTestCase(), cleanup()
    */
    function cleanupTestCase() {}

    /*!
        \qmlmethod TestCase::init()

        This function is called before each test function that is
        executed in the \l TestCase type.  The default implementation
        does nothing.  The application can provide its own implementation
        to perform initialization before each test function.

        \sa cleanup(), initTestCase()
    */
    function init() {}

    /*!
        \qmlmethod TestCase::cleanup()

        This function is called after each test function that is
        executed in the \l TestCase type.  The default implementation
        does nothing.  The application can provide its own implementation
        to perform cleanup after each test function.

        \sa init(), cleanupTestCase()
    */
    function cleanup() {}

    /*! \internal */
    function qtest_runInternal(prop, arg) {
        try {
            qtest_testCaseResult = testCase[prop](arg)
        } catch (e) {
            qtest_testCaseResult = []
            if (e.message.indexOf("QtQuickTest::") != 0) {
                // Test threw an unrecognized exception - fail.
                qtest_results.fail("Uncaught exception: " + e.message,
                             e.fileName, e.lineNumber)
            }
        }
        return !qtest_results.failed
    }

    /*! \internal */
    function qtest_runFunction(prop, arg) {
        qtest_runInternal("init")
        if (!qtest_results.skipped) {
            qtest_runInternal(prop, arg)
            qtest_results.finishTestData()
            qtest_runInternal("cleanup")
            qtest_results.finishTestDataCleanup()
            // wait(0) will call processEvents() so objects marked for deletion
            // in the test function will be deleted.
            wait(0)
        }
    }

    /*! \internal */
    function qtest_runBenchmarkFunction(prop, arg) {
        qtest_results.startMeasurement()
        do {
            qtest_results.beginDataRun()
            do {
                // Run the initialization function.
                qtest_runInternal("init")
                if (qtest_results.skipped)
                    break

                // Execute the benchmark function.
                if (prop.indexOf("benchmark_once_") != 0)
                    qtest_results.startBenchmark(TestResult.RepeatUntilValidMeasurement, qtest_results.dataTag)
                else
                    qtest_results.startBenchmark(TestResult.RunOnce, qtest_results.dataTag)
                while (!qtest_results.isBenchmarkDone()) {
                    var success = qtest_runInternal(prop, arg)
                    qtest_results.finishTestData()
                    if (!success)
                        break
                    qtest_results.nextBenchmark()
                }
                qtest_results.stopBenchmark()

                // Run the cleanup function.
                qtest_runInternal("cleanup")
                qtest_results.finishTestDataCleanup()
                // wait(0) will call processEvents() so objects marked for deletion
                // in the test function will be deleted.
                wait(0)
            } while (!qtest_results.measurementAccepted())
            qtest_results.endDataRun()
        } while (qtest_results.needsMoreMeasurements())
    }

    /*! \internal */
    function qtest_run() {
        if (util.printAvailableFunctions) {
            completed = true
            return
        }

        if (TestLogger.log_start_test()) {
            qtest_results.reset()
            qtest_results.testCaseName = name
            qtest_results.startLogging()
        } else {
            qtest_results.testCaseName = name
        }
        running = true

        // Check the run list to see if this class is mentioned.
        var functionsToRun = qtest_results.functionsToRun
        if (functionsToRun.length > 0) {
            var found = false
            var list = []
            if (name.length > 0) {
                var prefix = name + "::"
                for (var index in functionsToRun) {
                    if (functionsToRun[index].indexOf(prefix) == 0) {
                        list.push(functionsToRun[index])
                        found = true
                    }
                }
            }
            if (!found) {
                completed = true
                if (!TestLogger.log_complete_test(qtest_testId)) {
                    qtest_results.stopLogging()
                    Qt.quit()
                }
                qtest_results.testCaseName = ""
                return
            }
            functionsToRun = list
        }

        // Run the initTestCase function.
        qtest_results.functionName = "initTestCase"
        var runTests = true
        if (!qtest_runInternal("initTestCase"))
            runTests = false
        qtest_results.finishTestData()
        qtest_results.finishTestDataCleanup()
        qtest_results.finishTestFunction()

        // Run the test methods.
        var testList = []
        if (runTests) {
            for (var prop in testCase) {
                if (prop.indexOf("test_") != 0 && prop.indexOf("benchmark_") != 0)
                    continue
                var tail = prop.lastIndexOf("_data");
                if (tail != -1 && tail == (prop.length - 5))
                    continue
                testList.push(prop)
            }
            testList.sort()
        }
        var checkNames = (functionsToRun.length > 0)
        for (var index in testList) {
            var prop = testList[index]
            var datafunc = prop + "_data"
            var isBenchmark = (prop.indexOf("benchmark_") == 0)
            if (checkNames) {
                var index = functionsToRun.indexOf(name + "::" + prop)
                if (index < 0)
                    continue
                functionsToRun.splice(index, 1)
            }
            qtest_results.functionName = prop

            if (!(datafunc in testCase))
                datafunc = "init_data";

            if (datafunc in testCase) {
                if (qtest_runInternal(datafunc)) {
                    var table = qtest_testCaseResult
                    var haveData = false
                    qtest_results.initTestTable()
                    for (var index in table) {
                        haveData = true
                        var row = table[index]
                        if (!row.tag)
                            row.tag = "row " + index    // Must have something
                        qtest_results.dataTag = row.tag
                        if (isBenchmark)
                            qtest_runBenchmarkFunction(prop, row)
                        else
                            qtest_runFunction(prop, row)
                        qtest_results.dataTag = ""
                    }
                    if (!haveData) {
                        if (datafunc === "init_data")
                           qtest_runFunction(prop, null, isBenchmark)
                        else
                           qtest_results.warn("no data supplied for " + prop + "() by " + datafunc + "()"
                                            , util.callerFile(), util.callerLine());
                    }
                    qtest_results.clearTestTable()
                }
            } else if (isBenchmark) {
                qtest_runBenchmarkFunction(prop, null, isBenchmark)
            } else {
                qtest_runFunction(prop, null, isBenchmark)
            }
            qtest_results.finishTestFunction()
            qtest_results.skipped = false
        }

        // Run the cleanupTestCase function.
        qtest_results.skipped = false
        qtest_results.functionName = "cleanupTestCase"
        qtest_runInternal("cleanupTestCase")

        // Complain about missing functions that we were supposed to run.
        if (functionsToRun.length > 0)
            qtest_results.fail("Could not find functions: " + functionsToRun, "", 0)

        // Clean up and exit.
        running = false
        completed = true
        qtest_results.finishTestData()
        qtest_results.finishTestDataCleanup()
        qtest_results.finishTestFunction()
        qtest_results.functionName = ""

        // Stop if there are no more tests to be run.
        if (!TestLogger.log_complete_test(qtest_testId)) {
            qtest_results.stopLogging()
            Qt.quit()
        }
        qtest_results.testCaseName = ""
    }

    onWhenChanged: {
        if (when != qtest_prevWhen) {
            qtest_prevWhen = when
            if (when && !completed && !running && qtest_componentCompleted)
                qtest_run()
        }
    }

    onOptionalChanged: {
        if (!completed) {
            if (optional)
                TestLogger.log_optional_test(qtest_testId)
            else
                TestLogger.log_mandatory_test(qtest_testId)
        }
    }


    Component.onCompleted: {
        QTestRootObject.hasTestCase = true;
        qtest_componentCompleted = true;

        if (util.printAvailableFunctions) {
            var testList = []
            for (var prop in testCase) {
                if (prop.indexOf("test_") != 0 && prop.indexOf("benchmark_") != 0)
                    continue
                var tail = prop.lastIndexOf("_data");
                if (tail != -1 && tail == (prop.length - 5))
                    continue
                // Note: cannot run functions in TestCase elements
                // that lack a name.
                if (name.length > 0)
                    testList.push(name + "::" + prop + "()")
            }
            testList.sort()
            for (var index in testList)
                console.log(testList[index])
            return
        }
        qtest_testId = TestLogger.log_register_test(name)
        if (optional)
            TestLogger.log_optional_test(qtest_testId)
        qtest_prevWhen = when
        if (when && !completed && !running)
            qtest_run()
    }
}
