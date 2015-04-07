## Asus Xtion Intrinsic Calibration Parameters
These parameters are the result of intrinsic calibration performed for our asus xtion sensors. 
Since multiple sensors are present, each one is labelled with a number 
and each one should have separate intrinsic parameters.

- asus1 identifies Centro Piaggio sensor, the one used by Poses Scanner and Kuka head.
- asus2 identifies Phase Space lab sensor.

### Usage
To use these instrinsics into your program you should create or modify a launch file that calls `openni2_launch` this way:

- add `<arg name="asus" default="asus1" />` to launch file configuration. Note, replace `asus1` with the sensor you want.
- add these lines into the launch file implementation:

`<include file="$(find openni2_launch)/launch/openni2_launch">`

`<arg name="rgb_camera_info_url" value="file:///$(find calibration)/config/intrinsics/rgb_$(arg asus).yaml"/>`

`<arg name="depth_camera_info_url" value="file:///$(find calibration)/config/intrinsics/depth_$(arg asus).yaml"/>`

`</include>`

Now launch your custom launch file and you should also see a confirmation from `openni2_launch` that 
you are loading the above intrinsics.
