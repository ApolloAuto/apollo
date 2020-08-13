^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tf2
^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.16 (2017-07-14)
-------------------
* remove explicit templating to standardize on overloading. But provide backwards compatibility with deprecation.
* Merge pull request `#144 <https://github.com/ros/geometry2/issues/144>`_ from clearpathrobotics/dead_lock_fix
  Solve a bug that causes a deadlock in MessageFilter
* Resolve 2 places where the error_msg would not be propogated.
  Fixes `#198 <https://github.com/ros/geometry2/issues/198>`_
* Remove generate_rand_vectors() from a number of tests. (`#227 <https://github.com/ros/geometry2/issues/227>`_)
* fixing include directory order to support overlays (`#231 <https://github.com/ros/geometry2/issues/231>`_)
* replaced dependencies on tf2_msgs_gencpp by exported dependencies
* Document the lifetime of the returned reference for getFrameId getTimestamp
* relax normalization tolerance. `#196 <https://github.com/ros/geometry2/issues/196>`_ was too strict for some use cases. (`#220 <https://github.com/ros/geometry2/issues/220>`_)
* Solve a bug that causes a deadlock in MessageFilter
* Contributors: Adel Fakih, Chris Lalancette, Christopher Wecht, Tully Foote, dhood

0.5.15 (2017-01-24)
-------------------

0.5.14 (2017-01-16)
-------------------
* fixes `#194 <https://github.com/ros/geometry2/issues/194>`_ check for quaternion normalization before inserting into storage (`#196 <https://github.com/ros/geometry2/issues/196>`_)
  * check for quaternion normalization before inserting into storage
  * Add test to check for transform failure on invalid quaternion input
* updating getAngleShortestPath() (`#187 <https://github.com/ros/geometry2/issues/187>`_)
* Move internal cache functions into a namespace
  Fixes https://github.com/ros/geometry2/issues/175
* Link properly to convert.h
* Landing page for tf2 describing the conversion interface
* Fix comment on BufferCore::MAX_GRAPH_DEPTH.
* Contributors: Jackie Kay, Phil Osteen, Tully Foote, alex, gavanderhoorn

0.5.13 (2016-03-04)
-------------------

0.5.12 (2015-08-05)
-------------------
* add utilities to get yaw, pitch, roll and identity transform
* provide more conversions between types
  The previous conversion always assumed that it was converting a
  non-message type to a non-message type. Now, one, both or none
  can be a message or a non-message.
* Contributors: Vincent Rabaud

0.5.11 (2015-04-22)
-------------------

0.5.10 (2015-04-21)
-------------------
* move lct_cache into function local memoryfor `#92 <https://github.com/ros/geometry_experimental/issues/92>`_
* Clean up range checking. Re: `#92 <https://github.com/ros/geometry_experimental/issues/92>`_
* Fixed chainToVector
* release lock before possibly invoking user callbacks. Fixes `#91 <https://github.com/ros/geometry_experimental/issues/91>`_
* Contributors: Jackie Kay, Tully Foote

0.5.9 (2015-03-25)
------------------
* fixing edge case where two no frame id lookups matched in getLatestCommonTime
* Contributors: Tully Foote

0.5.8 (2015-03-17)
------------------
* change from default argument to overload to avoid linking issue `#84 <https://github.com/ros/geometry_experimental/issues/84>`_
* remove useless Makefile files
* Remove unused assignments in max/min functions
* change _allFramesAsDot() -> _allFramesAsDot(double current_time)
* Contributors: Jon Binney, Kei Okada, Tully Foote, Vincent Rabaud

0.5.7 (2014-12-23)
------------------

0.5.6 (2014-09-18)
------------------

0.5.5 (2014-06-23)
------------------
* convert to use console bridge from upstream debian package https://github.com/ros/rosdistro/issues/4633
* Fix format string
* Contributors: Austin, Tully Foote

0.5.4 (2014-05-07)
------------------
* switch to boost signals2 following `ros/ros_comm#267 <https://github.com/ros/ros_comm/issues/267>`_, blocking `ros/geometry#23 <https://github.com/ros/geometry/issues/23>`_
* Contributors: Tully Foote

0.5.3 (2014-02-21)
------------------

0.5.2 (2014-02-20)
------------------

0.5.1 (2014-02-14)
------------------

0.5.0 (2014-02-14)
------------------

0.4.10 (2013-12-26)
-------------------
* updated error message. fixes `#38 <https://github.com/ros/geometry_experimental/issues/38>`_
* tf2: add missing console bridge include directories (fix `#48 <https://github.com/ros/geometry_experimental/issues/48>`_)
* Fix const correctness of tf2::Vector3 rotate() method
  The method does not modify the class thus should be const.
  This has already been fixed in Bullet itself.
* Contributors: Dirk Thomas, Timo Rohling, Tully Foote

0.4.9 (2013-11-06)
------------------

0.4.8 (2013-11-06)
------------------
* moving python documentation to tf2_ros from tf2 to follow the code
* removing legacy rospy dependency. implementation removed in 0.4.0 fixes `#27 <https://github.com/ros/geometry_experimental/issues/27>`_

0.4.7 (2013-08-28)
------------------
* switching to use allFramesAsStringNoLock inside of getLatestCommonTime and walkToParent and locking in public API _getLatestCommonTime instead re `#23 <https://github.com/ros/geometry_experimental/issues/23>`_
* Fixes a crash in tf's view_frames related to dot code generation in allFramesAsDot

0.4.6 (2013-08-28)
------------------
* cleaner fix for `#19 <https://github.com/ros/geometry_experimental/issues/19>`_
* fix pointer initialization.  Fixes `#19 <https://github.com/ros/geometry_experimental/issues/19>`_
* fixes `#18 <https://github.com/ros/geometry_experimental/issues/18>`_ for hydro
* package.xml: corrected typo in description

0.4.5 (2013-07-11)
------------------
* adding _chainAsVector method for https://github.com/ros/geometry/issues/18
* adding _allFramesAsDot for backwards compatability https://github.com/ros/geometry/issues/18

0.4.4 (2013-07-09)
------------------
* making repo use CATKIN_ENABLE_TESTING correctly and switching rostest to be a test_depend with that change.
* tf2: Fixes a warning on OS X, but generally safer
  Replaces the use of pointers with shared_ptrs,
  this allows the polymorphism and makes it so that
  the compiler doesn't yell at us about calling
  delete on a class with a public non-virtual
  destructor.
* tf2: Fixes compiler warnings on OS X
  This exploited a gcc specific extension and is not
  C++ standard compliant. There used to be a "fix"
  for OS X which no longer applies. I think it is ok
  to use this as an int instead of a double, but
  another way to fix it would be to use a define.
* tf2: Fixes linkedit errors on OS X

0.4.3 (2013-07-05)
------------------

0.4.2 (2013-07-05)
------------------
* adding getCacheLength() to parallel old tf API
* removing legacy static const variable MAX_EXTRAPOLATION_DISTANCE copied from tf unnecessesarily

0.4.1 (2013-07-05)
------------------
* adding old style callback notifications to BufferCore to enable backwards compatability of message filters
* exposing dedicated thread logic in BufferCore and checking in Buffer
* more methods to expose, and check for empty cache before getting latest timestamp
* adding methods to enable backwards compatability for passing through to tf::Transformer

0.4.0 (2013-06-27)
------------------
* splitting rospy dependency into tf2_py so tf2 is pure c++ library.
* switching to console_bridge from rosconsole
* moving convert methods back into tf2 because it does not have any ros dependencies beyond ros::Time which is already a dependency of tf2
* Cleaning up unnecessary dependency on roscpp
* Cleaning up packaging of tf2 including:
  removing unused nodehandle
  fixing overmatch on search and replace
  cleaning up a few dependencies and linking
  removing old backup of package.xml
  making diff minimally different from tf version of library
* suppressing bullet LinearMath copy inside of tf2, so it will not collide, and should not be used externally.
* Restoring test packages and bullet packages.
  reverting 3570e8c42f9b394ecbfd9db076b920b41300ad55 to get back more of the packages previously implemented
  reverting 04cf29d1b58c660fdc999ab83563a5d4b76ab331 to fix `#7 <https://github.com/ros/geometry_experimental/issues/7>`_
* fixing includes in unit tests
* Make PythonLibs find_package python2 specific
  On systems with python 3 installed and default, find_package(PythonLibs) will find the python 3 paths and libraries. However, the c++ include structure seems to be different in python 3 and tf2 uses includes that are no longer present or deprecated.
  Until the includes are made to be python 3 compliant, we should specify that the version of python found must be python 2.

0.3.6 (2013-03-03)
------------------

0.3.5 (2013-02-15 14:46)
------------------------
* 0.3.4 -> 0.3.5

0.3.4 (2013-02-15 13:14)
------------------------
* 0.3.3 -> 0.3.4
* moving LinearMath includes to include/tf2

0.3.3 (2013-02-15 11:30)
------------------------
* 0.3.2 -> 0.3.3
* fixing include installation of tf2

0.3.2 (2013-02-15 00:42)
------------------------
* 0.3.1 -> 0.3.2
* fixed missing include export & tf2_ros dependecy

0.3.1 (2013-02-14)
------------------
* 0.3.0 -> 0.3.1
* fixing PYTHON installation directory

0.3.0 (2013-02-13)
------------------
* switching to version 0.3.0
* adding setup.py to tf2 package
* fixed tf2 exposing python functionality
* removed line that was killing tf2_ros.so
* fixing catkin message dependencies
* removing packages with missing deps
* adding missing package.xml
* adding missing package.xml
* adding missing package.xml
* catkinizing geometry-experimental
* removing bullet headers from use in header files
* removing bullet headers from use in header files
* merging my recent changes
* setting child_frame_id overlooked in revision 6a0eec022be0 which fixed failing tests
* allFramesAsString public and internal methods seperated.  Public method is locked, private method is not
* fixing another scoped lock
* fixing one scoped lock
* fixing test compilation
* merge
* Error message fix, ros-pkg5085
* Check if target equals to source before validation
* When target_frame == source_frame, just returns an identity transform.
* adding addition ros header includes for strictness
* Fixed optimized lookups with compound transforms
* Fixed problem in tf2 optimized branch. Quaternion multiplication order was incorrect
* fix compilation on 32-bit
* Josh fix: Final inverse transform composition (missed multiplying the sourcd->top vector by the target->top inverse orientation). b44877d2b054
* Josh change: fix first/last time case. 46bf33868e0d
* fix transform accumulation to parent
* fix parent lookup, now works on the real pr2's tree
* move the message filter to tf2_ros
* tf2::MessageFilter + tests.  Still need to change it around to pass in a callback queue, since we're being triggered directly from the tf2 buffer
* Don't add the request if the transform is already available.  Add some new tests
* working transformable callbacks with a simple (incomplete) test case
* first pass at a transformable callback api, not tested yet
* add interpolation cases
* fix getLatestCommonTime -- no longer returns the latest of any of the times
* Some more optimization -- allow findClosest to inline
* another minor speedup
* Minorly speed up canTransform by not requiring the full data lookup, and only looking up the parent
* Add explicit operator= so that we can see the time in it on a profile graph.  Also some minor cleanup
* minor cleanup
* add 3 more cases to the speed test
* Remove use of btTransform at all from transform accumulation, since the conversion to/from is unnecessary, expensive, and can introduce floating point error
* Don't use btTransform as an intermediate when accumulating transforms, as constructing them takes quite a bit of time
* Completely remove lookupLists().  canTransform() now uses the same walking code as lookupTransform().  Also fixed a bug in the static transform publisher test
* Genericise the walk-to-top-parent code in lookupTransform so that it will be able to be used by canTransform as well (minus the cost of actually computing the transform)
* remove id lookup that wasn't doing anything
* Some more optimization:
  * Reduce # of TransformStorage copies made in TimeCache::getData()
  * Remove use of lookupLists from getLatestCommonTime
* lookupTransform() no longer uses lookupLists unless it's called with Time(0).  Removes lots of object construction/destruction due to removal of pushing back on the lists
* Remove CompactFrameID in favor of a typedef
* these mode checks are no longer necessary
* Fix crash when testing extrapolation on the forward transforms
* Update cache unit tests to work with the changes TransformStorage.
  Also make sure that BT_USE_DOUBLE_PRECISION is set for tf2.
* remove exposure of time_cache.h from buffer_core.h
* Removed the mutex from TimeCache, as it's unnecessary (BufferCore needs to have its own mutex locked anyway), and this speeds things up by about 20%
  Also fixed a number of thread-safety problems
* Optimize test_extrapolation a bit, 25% speedup of lookupTransform
* use a hash map for looking up frame numbers, speeds up lookupTransform by ~8%
* Cache vectors used for looking up transforms.  Speeds up lookupTransform by another 10%
* speed up lookupTransform by another 25%
* speed up lookupTransform by another 2x.  also reduces the memory footprint of the cache significantly
* sped up lookupTransform by another 2x
* First add of a simple speed test
  Sped up lookupTransform 2x
* roscpp dependency explicit, instead of relying on implicit
* static transform tested and working
* tests passing and all throw catches removed too\!
* validating frame_ids up front for lookup exceptions
* working with single base class vector
* tests passing for static storage
* making method private for clarity
* static cache implementation and test
* cleaning up API doc typos
* sphinx docs for Buffer
* new dox mainpage
* update tf2 manifest
* commenting out twist
* Changed cache_time to cache_time_ to follow C++ style guide, also initialized it to actually get things to work
* no more rand in cache tests
* Changing tf2_py.cpp to use underscores instead of camelCase
* removing all old converter functions from transform_datatypes.h
* removing last references to transform_datatypes.h in tf2
* transform conversions internalized
* removing unused datatypes
* copying bullet transform headers into tf2 and breaking bullet dependency
* merge
* removing dependency on tf
* removing include of old tf from tf2
* update doc
* merge
* kdl unittest passing
* Spaces instead of tabs in YAML grrrr
* Adding quotes for parent
* canTransform advanced ported
* Hopefully fixing YAML syntax
* new version of view_frames in new tf2_tools package
* testing new argument validation and catching bug
* Python support for debugging
* merge
* adding validation of frame_ids in queries with warnings and exceptions where appropriate
* Exposing ability to get frames as a string
* A compiling version of YAML debugging interface for BufferCore
* placeholder for tf debug
* fixing tf:: to tf2:: ns issues and stripping slashes on set in tf2 for backwards compatiabily
* Adding a python version of the BufferClient
* moving test to new package
* merging
* working unit test for BufferCore::lookupTransform
* removing unused method test and converting NO_PARENT test to new API
* Adding some comments
* Moving the python bindings for tf2 to the tf2 package from the tf2_py package
* buffercore tests upgraded
* porting tf_unittest while running incrmentally instead of block copy
* BufferCore::clear ported forward
* successfully changed lookupTransform advanced to new version
* switching to new implementation of lookupTransform tests still passing
* compiling lookupTransform new version
* removing tf_prefix from BufferCore.  BuferCore is independent of any frame_ids.  tf_prefix should be implemented at the ROS API level.
* initializing tf_prefix
* adding missing initialization
* suppressing warnings
* more tests ported
* removing tests for apis not ported forward
* setTransform tests ported
* old tests in new package passing due to backwards dependency.  now for the fun, port all 1500 lines :-)
* setTransform working in new framework as well as old
* porting more methods
* more compatability
* bringing in helper functions for buffer_core from tf.h/cpp
* rethrowing to new exceptions
* converting Storage to geometry_msgs::TransformStamped
* removing deprecated useage
* cleaning up includes
* moving all implementations into cpp file
* switching test to new class from old one
* Compiling version of the buffer client
* moving listener to tf_cpp
* removing listener, it should be in another package
* most of listener
* add cantransform implementation
* removing deprecated API usage
* initial import of listener header
* move implementation into library
* 2 tests of buffer
* moving executables back into bin
* compiling again with new design
* rename tfcore to buffercore
* almost compiling version of template code
* compiling tf2_core simple test
* add test to start compiling
* copying in tf_unittest for tf_core testing template
* prototype of tf2_core implemented using old tf.
* first version of template functions
* remove timeouts
* properly naming tf2_core.h from tf_core.h
* working cache test with tf2 lib
* first unit test passing, not yet ported
* tf_core api
* tf2 v2
* aborting port
* moving across time cache tf and datatypes headers
* copying exceptions from tf
* switching to tf2 from tf_core
*
