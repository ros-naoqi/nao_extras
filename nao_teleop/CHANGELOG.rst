^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package nao_teleop
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.1 (2015-08-11)
------------------
* publish only when move not zero
* Contributors: Karsten Knese

0.3.0 (2015-07-31)
------------------
* get code to use naoqi_bridge_msgs and not naoqi_msgs
* Contributors: Vincent Rabaud

0.2.2 (2014-11-13)
------------------
* comply to the new naoqi organization
* Contributors: Vincent Rabaud

0.2.1 (2014-09-07)
------------------
* Fix package.xml files
* Catkinization of nao_extras, based on commit cb7aedce3c7a3ea3391319f0b7e9aeb78ed13233
* Contributors: Armin Hornung

0.2.0 (2013-10-26)
------------------
* Drop nao_teleop support for old versions of ROS, no more "using..." in header
* nao_teleop: remove dependency on nao_remote
* Moved nao_teleop's yaml file to config dir
* Drop nao_teleop support for old versions of ROS
* Contributors: Armin Hornung, Miguel Sarabia

0.1.0 (2013-07-30)
------------------
* callBodyPoseClient offers interface to executing poses in nao_teleop (feature req. `#4 <https://github.com/ros-nao/nao_extras/issues/4>`_)
* added nao_teleop namespace, ROS CPP style formatting
* Applied patch from request `#3 <https://github.com/ros-nao/nao_extras/issues/3>`_: Restructure of nao_teleop to allow reuse via subclasses (thx to Nick Hawkes)
* fixes in nao_common teleop and pose_manager: check if action servers available, timeout
* nao_teleop: Adapted launch files to new package structure
* new package nao_teleop for teleoperation (moved from nao_remote)
* Contributors: Armin Hornung, Stefan Osswald
