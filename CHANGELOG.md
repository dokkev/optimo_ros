<!-- 
## Unreleased
##### Added
- Initial commit with optimo_quick_start_app.
- Added bump_version.py to bump the version of the project.
- Added the Frontend GUI for optimo_quick_start_app.
- Added Release pipeline.
- Added optimo_msgs package.
- Added optimo_api package.
- Added optimo_sim package.
- Added optimo_moveit_config package.
- Added optimo_description package.
- Added optimo_hardware package.
- Added optimo_controllers package.
- Added optimo_bringup package.
- Added cartesian pose servoing to ServoCallback.
- Added support for nested namespaces, in preparation for the Dex robot using this as a dependency.
- Added prefix and standalone tags to the urdf, in preparation for a left and right arm for the Dex.
- Added left and right mounted yaml files for simulation.
- Added support for disabling the robot from Model, using model_safety_error.
- Added a service to calculate load on the end effector.
- Robot index 1 now launches 10 seconds later, to stagger ethercat activity.

##### Removed
- Removed Ubuntu 22.04 and ROS2 Humble installation instructions from the README.md.
- Removed Moveit from demo launch due to issues with namespacing, to be worked on later.
##### Fixed
- Updated README.md to include the new bump_version.py script, release instructions.
- Fixed pipeline to show test results correctly
- Fixed some build warnings for optimo_quick_start_app.
- bump_version.py now also updates the major and minor version in package.xml files depending on the args.
- Cleaned up README.md to only have instructions for development. Production instructions belong on the production repositories.
- stop_cb now return true with a message if the robot is not currently in an active callback
- Minor tuning of MoveIt servo.
- Fixed the demo launch to include rviz. 
-Reworked simulation to be a launch flag instead of involving separate launch files
- rviz_rsp is now the launch file that launches the joint state gui for visualization of the urdf
- Made various changes to yaml and xacro files to reflect actual Dex hardware.
- Temporarily fixed a bug where two instances of servocallback would fail when launched together, using a mutex. This implies the need for a more robust fix for running multiple arms in general.  
- Fixed a bug to allow grippers to use the proper CAN transport.
- Fixed a bug where restarting servo callback would result in the robot continuing to move to the last target pose.
- Fixed a bug where stop_cb would not result in the robot stopping all callbacks.
- Fixed a bug where the robot would not stop callbacks after it was disabled, resulting in the robot restarting immediately upon re-enabling.

##### Internal
 -->

## Released
### 0.1
##### Added
##### Removed
##### Fixed
##### Internal