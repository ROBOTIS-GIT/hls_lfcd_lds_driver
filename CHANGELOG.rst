^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hls_lfcd_lds_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.0 (2019-08-20)
------------------
* Supported ROS 2 Dashing Diademata
* Added parameter for initilization and modified qos
* Reduced scope to boost-system dependency
* Fixed build warnings in ROS2 Dashing
* Contributors: Emerson Knapp, Darby Lim, Pyo

1.1.0 (2019-01-23)
------------------
* Added lpthread library in Makefile
* Added CI for ROS Melodic
* Modified hlds_laser_segment_publisher
* Contributors: Gilbert, Pyo

1.0.0 (2018-05-29)
------------------
* Modified max-range to avoid fencepost error.
* Merged pull request `#27 <https://github.com/ROBOTIS-GIT/hls_lfcd_lds_driver/issues/27>`_ from skasperski/master
  Corrected max-range to avoid fencepost error.
* Merged pull request `#29 <https://github.com/ROBOTIS-GIT/hls_lfcd_lds_driver/issues/29>`_ `#28 <https://github.com/ROBOTIS-GIT/hls_lfcd_lds_driver/issues/28>`_ `#24 <https://github.com/ROBOTIS-GIT/hls_lfcd_lds_driver/issues/24>`_
* Contributors: Gilbert, Sebastian Kasperski, Pyo

0.1.5 (2018-03-14)
------------------
* Added non-ROS applications for LDS (linux, windows, mac)
* Added a link for e-Manual
* Added .travis.yml for Travis CI
* Modified for firmware upgrade
* Refactoring for release
* Contributors: Darby Lim, Pyo

0.1.4 (2017-09-14)
------------------
* Modified to stop motor of LDS
* Deleted unused lfcdstartstop arg of LFCDLaser. Fix an exception error.
* Merged pull request `#8 <https://github.com/ROBOTIS-GIT/hls_lfcd_lds_driver/issues/8>`_ from hara-y/stop_motor_in_dtor
* Fixed code format
* Contributors: Yoonseok Pyo, Yoshitaka HARA

0.1.3 (2017-08-18)
------------------
* Fixed a bug of beam index [0, 0, 1, ..., 358] > [0, 1, 2, ..., 359]
* Merged pull request `#7 <https://github.com/ROBOTIS-GIT/hls_lfcd_lds_driver/pull/7>`_ from hara-y
* Contributors: Darby Lim, Yoshitaka HARA

0.1.2 (2017-04-24)
------------------
* Modified the timing to take a timestamp for sync the scan and odom
* Added a hlds_laser_segment_publisher node
* Contributors: Pyo

0.1.1 (2016-12-06)
------------------
* Modified the package name and related codes
* Added a ROS package for HLDS Laser Distance Sensor
* Contributors: JH Yang, SP Kong, Pyo
