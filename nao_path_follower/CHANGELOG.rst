^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package nao_path_follower
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.1 (2015-08-11)
------------------

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
* Fixing oscillations in path follower when next target is behind robot
* Add missing parameter for damping yaw velocity
* Small cleanup of nao_path_follower, fix potential problems with std::distance
  and angular distance (+-)
* Merge pull request `#3 <https://github.com/ros-nao/nao_extras/issues/3>`_ from sboettcher/groovy-devel
  path_follower now recognizes a straight line in a path and offsets the walk target further ahead
* Removing unneeded workaround for NaoQI API < 1.12
* Cleaning up unused code
* Cleaning up nao_path_follower, more parameters
* Fixing nao_path_follower for paths with a single pose
* Small fix in nao_path_follower package.xml
* Cleaning up nao_path_follower output
* Fix package.xml files
* Catkinization of nao_extras, based on commit cb7aedce3c7a3ea3391319f0b7e9aeb78ed13233
* Contributors: Armin Hornung

0.2.0 (2013-10-26)
------------------
* nao_remote renamed to nao_path_follower, now only contains the path
  follower node
* Contributors: Armin Hornung

0.1.0 (2013-07-30)
------------------
