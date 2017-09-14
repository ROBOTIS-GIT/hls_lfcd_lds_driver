^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hls_lfcd_lds_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.4 (2017-09-14)
-----------
* modify to stop motor of LDS
* delete unused lfcdstartstop arg of LFCDLaser. Fix an exception error.
* merge pull request `#8 <https://github.com/ROBOTIS-GIT/hls_lfcd_lds_driver/issues/8>`_ from hara-y/stop_motor_in_dtor
* fixed code format
* Contributors: Yoonseok Pyo, Yoshitaka HARA

0.1.3 (2017-08-18)
-----------
* fixed a bug of beam index [0, 0, 1, ..., 358] > [0, 1, 2, ..., 359]
* merge pull request `#7 <https://github.com/ROBOTIS-GIT/hls_lfcd_lds_driver/pull/7>`_ from hara-y
* Contributors: Darby Lim, Yoshitaka HARA

0.1.2 (2017-04-24)
-----------
* modified the timing to take a timestamp for sync the scan and odom
* added a hlds_laser_segment_publisher node
* Contributors: Pyo

0.1.1 (2016-12-06)
-----------
* modified the package name and related codes
* added a ROS package for HLDS Laser Distance Sensor
* Contributors: JH Yang, SP Kong, Pyo
