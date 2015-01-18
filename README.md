calibration
===========

A general calibration tool for the equipments in Centro Piaggio.

The hand-eye calibration problem first appeared and got its name from the robotics community, where a camera ("eye") was mounted on the gripper ("hand") of a robot. The cameras was calibrated using a calibration pattern. Then the unknown transformation from the robot coordinate system to the calibration pattern coordinate system as well as the transformation from the camera to the hand ocordinate system need to be estimated simultaneously.

For more details on the problem, check: http://campar.in.tum.de/Chair/HandEyeCalibration

Dependencies
------------

`sudo apt-get install ros-indigo-ar-track-alvar ros-indigo-ar-track-alvar-msgs ros-indigo-ar-track-alvar-meta`

Mostly, we are using the Asus, but this package can be easily extended to consider other camera types. Thus, for the Asus, you need:

`sudo apt-get install ros-indigo-openni2-launch ros-indigo-openni2-camera`

And depending which of the cases below you are using, you might need the respective packages within.

KUKA LWR and Asus in-hand and head calibratrion
-----------------------------------------------




Asus phase-space calibration
----------------------------

1. Ensure the ar marker is well centered in the calibrator object. The reference frame of both are coincident to facilitate the reasoning.

2. Follow the instructions in the phase space package to track the "calibrator" object. Remember to check that the led ids correspond exactly to the ones mounted on the calibrator.

3. `roslaunch calibration kinect_phase_space_calibration.launch`

4. Check on rviz that the calibrator is correctly being tracked by both systems, check the axes correspond to those of the pattern.

5. When you feel comfortable `rosservice call /calibrate` and don't move the calibrator.

The transformation between the two systems is being published, check the `kinect_phase_space_calibration.launch` the names of the frames being used.