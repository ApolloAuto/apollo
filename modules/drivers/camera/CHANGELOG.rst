^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package usb_cam
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.4 (2015-08-18)
------------------
* Installs launch files
* Merge pull request #37 from tzutalin/develop
  Add a launch file for easy test
* Add a launch file for easy test
* Contributors: Russell Toris, tzu.ta.lin

0.3.3 (2015-05-14)
------------------
* Merge pull request #36 from jsarrett/develop
  add gain parameter
* add gain parameter
* Contributors: James Sarrett, Russell Toris

0.3.2 (2015-03-24)
------------------
* Merge pull request #34 from eliasm/develop
  fixed check whether calibration file exists
* fixed check whether calibration file exists
* Contributors: Elias Mueggler, Russell Toris

0.3.1 (2015-02-20)
------------------
* Merge pull request #32 from kmhallen/mono8
  Publish YUVMONO10 images as mono8 instead of rgb8
* Publish YUVMONO10 images as mono8 instead of rgb8
* Contributors: Kevin Hallenbeck, Russell Toris

0.3.0 (2015-01-26)
------------------
* Merge pull request #30 from mitchellwills/develop
  Removed global state from usb_cam by encapsulating it inside an object
* Made device name a std::string instead of const char*
* Added usb_cam namespace
* Added underscore sufix to class fields
* Removed camera_ prefix from methods
* Moved methods to parse pixel_format and io_method from string to UsbCam
* Moved camera_image_t struct to be private in UsbCam
* Cleaned up parameter assignment
* Made set_v4l_parameters a non-static function
* Moved set_v4l_parameters to UsbCam object
* Removed global state from usb_cam by encapsulating it inside an object
  function and structions in usb_cam.h became public and everything else is private
* Merge pull request #28 from mitchellwills/develop
  Fix installation of header files
* Fix installation of header files
* Contributors: Mitchell Wills, Russell Toris

0.2.0 (2015-01-16)
------------------
* Bug fix in camera info settings.
* Update .travis.yml
* Merge pull request #27 from bosch-ros-pkg/default_camera_info
  sets default camera info
* sets default camera info
* Contributors: Russell Toris

0.1.13 (2014-12-02)
-------------------
* Merge pull request #25 from blutack/patch-1
  Warn rather than error if framerate can't be set
* Warn rather than error if framerate can't be set
  The driver doesn't currently work with em28xx based devices as they don't allow the framerate to be set directly and the node exits with an error. Changing to a warning allows these devices to be used.
* Update README.md
* Merge pull request #24 from rjw57/do-not-touch-parameters-unless-asked
  do not modify parameters unless explicitly set
* do not modify parameters unless explicitly set
  The contrast, saturation, brightness, sharpness and focus parameters
  were recently added to usb_cam. This caused a regression
  (sigproc/robotic_surgery#17) whereby the default settings for a webcam
  are overridden in all cases by the hard-coded defaults in usb_cam.
  In the absence of a know good set of "default" values, leave the
  parameters unset unless the user has explicitly set them in the launch
  file.
* Contributors: Rich Wareham, Russell Toris, blutack

0.1.12 (2014-11-05)
-------------------
* Merge pull request #22 from dekent/develop
  White balance parameters
* Parameter to enable/disable auto white balance
* Added parameters for white balance
* uses version major to check for av_codec
* uses version header to check for AV_CODEC_ID_MJPEG
* Contributors: David Kent, Russell Toris

0.1.11 (2014-10-30)
-------------------
* Merge pull request #20 from dekent/develop
  More Parameters
* bug fix
* Setting focus when autofocus is disabled
* Parameter adjusting
* Added parameter setting for absolute focus, brightness, contrast, saturation, and sharpness
* Contributors: David Kent, Russell Toris

0.1.10 (2014-10-24)
-------------------
* Merge pull request #19 from bosch-ros-pkg/av_codec_id
  Removed deprecated CODEC_ID
* added legacy macro constants for libav 10
* Renamed deprecated CODEC_ID constants to AV_CODEC_ID to fix compilation for libav 10
* Contributors: Andrzej Pronobis, Russell Toris

0.1.9 (2014-08-26)
------------------
* Uses ros::Rate to enforce software framerate instead of custom time check
* Merge pull request #16 from liangfok/feature/app_level_framerate_control
  Modified to enforce framerate control at the application level in additi...
* Modified to enforce framerate control at the application level in addition to at the driver level.  This is necessary since the drivers for my webcam did not obey the requested framerate.
* Contributors: Russell Toris, liang

0.1.8 (2014-08-21)
------------------
* autoexposure and exposure settings now exposed via ROS parameters
* added ability to call v4l-utils as well as correctly set autofocus
* cleanup of output
* Merge pull request #15 from mistoll/develop
  added support for RGB24 pixel format
* Added RGB24 as pixel format
* Contributors: Michael Stoll, Russell Toris

0.1.7 (2014-08-20)
------------------
* changelog fixed
* minor cleanup and ability to change camera name and info
* Contributors: Russell Toris

0.1.6 (2014-08-15)
------------------
* Merge pull request #14 from KaijenHsiao/master
  added support for 10-bit mono cameras advertising as YUV
* added support for 10-bit mono cameras advertising as YUV (such as Leopard Imaging's LI-USB30-V034)
* Update CHANGELOG.rst
* changelog updated
* Merge pull request #13 from vrabaud/develop
  add a a ros::spinOnce to get set_camera_info working
* add a a ros::spinOnce to get set_camera_info working
  This is explained in the docs of CameraInfoManager
  https://github.com/ros-perception/image_common/blob/hydro-devel/camera_info_manager/include/camera_info_manager/camera_info_manager.h#L71
  Also, this fixes https://github.com/ros-perception/image_pipeline/issues/78
* Contributors: Kaijen Hsiao, Russell Toris, Vincent Rabaud, sosentos

0.1.5 (2014-07-28)
------------------
* auto format
* cleanup of readme and such
* Merge branch 'hydro-devel' of github.com:bosch-ros-pkg/usb_cam
* Merge pull request #11 from pronobis/hydro-devel
  Fixed a bug with av_free missing by adding a proper include.
* Fixed a bug with av_free missing by adding a proper include on Ubuntu 14.04.
* Merge pull request #7 from cottsay/groovy-devel
  Use pkg-config to find avcodec and swscale
* Merge pull request #5 from FriedCircuits/hydro-devel
  Remove requirments for self_test
* Use pkg-config to find avcodec and swscale
* Update package.xml
* Remove selftest
* Remove selftest
* Update usb_cam_node.cpp
* Merge pull request #2 from jonbinney/7_17
  swap out deprecated libavcodec functions
* swap out deprecated libavcodec functions
* Contributors: Andrzej Pronobis, Jon Binney, Russell Toris, Scott K Logan, William

0.1.3 (2013-07-11)
------------------
* Merge pull request #1 from jonbinney/rosify
  Bag of improvements
* add framerate parameter
* use ROS_* for output
* use camera_info_manager
* Contributors: Jon Binney, Russell Toris

0.1.2 (2013-05-06)
------------------
* installs usb_cam_node
* Contributors: Russell Toris

0.1.1 (2013-05-02)
------------------
* cmake fixed
* ffmpeg added
* Contributors: Russell Toris

0.1.0 (2013-05-01)
------------------
* Update package.xml
* minor cleanup
* inital merge
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update CLONE_SETUP.sh
* Update README.md
* Updated the README.md.
* Updated the installation instructions.
* Fixed syntax in the README.
* Updated README for ARDUINO support.
* Fixed update script.
* Updated the readme and updating scripts.
* Updating for installation on Robot.
* Updated installs and README for ROS.
* Make sure the User knows to source the devel/setup.sh.
* Getting rid of subtrees and Catkinized USB CAM.
* Updating home to use ROSWS.
* Fixing the launch file for video1.
* Merge commit '0bc3322966e4c0ed259320827dd1f5cc8460efce'
  Conflicts:
  src/sofie_ros/package.xml
* Removed unnecessary file.
* Compiles.
* Adding the Catkin build scripts.
* Merge commit 'b2c739cb476e1e01425947e46dc2431464f241b3' as 'src/ar_track_alvar'
* Squashed 'src/ar_track_alvar/' content from commit 9ecca95
  git-subtree-dir: src/ar_track_alvar
  git-subtree-split: 9ecca9558edc7d3a9e692eacc93e082bf1e9a3e6
* Merge commit '9feb470d0ebdaa51e426be4d58f419b45928a671' as 'src/sofie_ros'
* Squashed 'src/sofie_ros/' content from commit 3ca5edf
  git-subtree-dir: src/sofie_ros
  git-subtree-split: 3ca5edfba496840b41bfe01dfdff883cacff1a97
* Removing stackts.
* Removing submodules.
* Fixed submodules.
* Removing old package.
* Merge branch 'catkin'
  Conflicts:
  README.md
  cmake_install.cmake
* Brancing package down to stack base.
* Catkininizing.
* (catkin)Catkininizing.
* Modifying the setup of roshome.
* Starting to Catkininize the project.
* (catkin)Starting to Catkininize the project.
* Going to catinize it.
* (catkin)Going to catinize it.
* Modified to new version of sofie_ros.
* Renamed import_csv_data.py to fileUtils.py, because it does more now.
* (catkin)Renamed import_csv_data.py to fileUtils.py, because it does more now.
* Updating to use a csv file specified by the user. Separating PyTables path manipulation into SOFIEHDFFORMAT.
* (catkin)Updating to use a csv file specified by the user. Separating PyTables path manipulation into SOFIEHDFFORMAT.
* Merge branch 'release/0.0.2'
* Created the install script.
* Removed the Python Packages as submodules.
* Merge branch 'release/0.0.1'
* Update the Git submodules.
* Modified the README and CLONE_SETUP.sh
* Added SOFIEHDFFORMAT as a submodule.
* Added the ExperimentControl Repo as a submodule.
* Working the CLONE install.
* Modifiying install script.
* Added a script to update the gitmodules for read-only clones.
* Merge branch 'master' of github.com:agcooke/roshome
* Initial commit
* Added the modules.
* Added usb_cam,
* Updating to Groovy.
* (catkin)Updating to Groovy.
* Added another potential launch file for exporting video from rosbag.
* (catkin)Added another potential launch file for exporting video from rosbag.
* Added a launcher to ros bag the usb_cam, for later playback.
* (catkin)Added a launcher to ros bag the usb_cam, for later playback.
* Added some files that were possibly not correct
* (catkin)Added some files that were possibly not correct
* Fixed bugs with the importing.
* (catkin)Fixed bugs with the importing.
* Added forgotten __init__.py file and changed to importdata sofiehdfformat funciton.
* (catkin)Added forgotten __init__.py file and changed to importdata sofiehdfformat funciton.
* Refractoring to make it possible to log to CSV.
  There were problems handling concurrent writing to
  pytables files. The package now logs to CSV and then
  provides a function to post import the data into
  SOFIEHDFFORMAT.
* (catkin)Refractoring to make it possible to log to CSV.
  There were problems handling concurrent writing to
  pytables files. The package now logs to CSV and then
  provides a function to post import the data into
  SOFIEHDFFORMAT.
* Exporting to a CSV. Does not work yet.
* (catkin)Exporting to a CSV. Does not work yet.
* Added a close on terminate signal handler.
* (catkin)Added a close on terminate signal handler.
* Made the marker size be set via a parameter to the launch file.
* (catkin)Made the marker size be set via a parameter to the launch file.
* Changed the Callibration data.
* (catkin)Changed the Callibration data.
* The ar_pose listener.
* (catkin)The ar_pose listener.
* Changed the sofie driver to directly safe the ar_pose data.
  We are going to perform experiments and this means that the extra
  data might be useful at a later stage.
* (catkin)Changed the sofie driver to directly safe the ar_pose data.
  We are going to perform experiments and this means that the extra
  data might be useful at a later stage.
* Changed the size of the marker.
* Updated the usb_cam config to work for home camera.
* Added callibration files and launch files.
* Turned off history.
* (catkin)Added some comments and renamed.
* Added some comments and renamed.
* (catkin)The Quaternions were mixed around. Fixed the launch file to log to file instead of screen.
* The Quaternions were mixed around. Fixed the launch file to log to file instead of screen.
* (catkin)Updating the README's.
* Updating the README's.
* Updated the launch file to launch ar_pose and rviz for debugging.
* (catkin)Added arguments to the launch script.
* Added arguments to the launch script.
* Added the Stack formating files.
* (catkin)Organising into a stack instead of separate packages.
* Organising into a stack instead of separate packages.
* Trying to figure out how to start and stop the node.
* Adding simple parameters.
* Added the ROS files.
* Basic driver now works for listening on a channel that broadcasts geometry_msgs.msg.QuaternionStamped messages.
* Working on the listerner that will write to HDFFormat.
* Creating a listerner that can write to sofiehdfformat files.
* Initial commit
* Contributors: Adrian Cooke, Russell Toris, ¨Adrian
