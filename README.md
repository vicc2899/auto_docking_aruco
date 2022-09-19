# Auto_Docking_Aruco_ROS

## Dependancies (already included in this github)

1) usb_cam <-- modified source code to flip the image (refer to ArUco Detection TARS pdf)
2) image_pipeline  
3) aruco_ros
4) tars_bringup

## Steps to make robot dock 1 time
### Step 1: Calibrate Camera (generates camera_info file in /home/.ros directory)
We will be using only one camera to track a aruco marker, hence we will be interested in monoculaar camera calibration method as instructed following. To run the cameracalibrator.py node for a monocular camera using an 8x6 chessboard with 108mm squares: (details [here](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration))
```
rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.108 image:=/usb_cam/image_raw camera:=/usb_cam
```

### Step 2: setup TARS movement
```
catkin_make -DCATKIN_WHITELIST_PACKAGES="lingao_msgs" 
catkin_make -DCATKIN_WHITELIST_PACKAGES=""
roscd lingao_base
sudo cp 50-tars-v4.rules /etc/udev/rules.d
```

### Step 3: launch TARS movement
```
roslaunch tars_bringup bringup.launch
```
### Step 4 (End) : launch full auto-docking code
```
roslaunch docking_movement dock.launch
```

## How to modify docking code

1. Import *Docking_OOP.py*
2. Create Dock_Robot object with x and y offset of camera
3. Run object.Start_Docking


### Example

```py
import Docking_OOP

if __name__=='__main__':
    rospy.init_node('docking_controller')
    x_offset=rospy.get_param("/x_offset")#use ROS_param to configure offsets...
    y_offset=rospy.get_param("/y_offset")
    TARS4=Docking_OOP.Dock_Robot(x_offset,y_offset)
    #or dont ¯\_(ツ)_/¯
    #TARS4=Dock_Robot(0.22,0) #x_offset,y_offset
    TARS4.Start_Docking()
```

Please see documentation for more information and modification.

## Steps to make robot generate test cases and auto-dock many times
### Modify dock.launch file to run dock_test_auto.py
Follow tars setup steps like before.
```
<launch>
    <node pkg="docking_movement" type="dock_test_auto" name="auto_docking_test_node" output="screen"></node>
    <param name="x_offset" value="0.22" /> <!-- x offset of camera to robot centre in meter --> 
    <param name="y_offset" value="0" /> <!-- y offset of camera to robot centre in meter --> 
    <include file="/home/nvidia/tars_ws/src/docking_movement/launch/usb_cam_stream_publisher.launch"/>
    <include file="/home/nvidia/tars_ws/src/docking_movement/launch/aruco_marker_finder.launch"/>


</launch>
```





## Modify test case generation file (gimme_mah_points.py)

Basic script to generate 9 testing points within specified angle and radius from the tag.

### Example

```py
import gimme_mah_points

if __name__=='__main__':
    rospy.init_node('test_points_generator')
    test1=gimme_mah_points.Test_Points(90,1.1,2.5,1)#max angle,minimum distance,maximum distance,robot offset from tag when parked
    print(test1.driving_instructions)
    print(test1.turning_instructions)
    #gives the turning angle and travel distance to reach each test points
    #-----------------------------------------------------------------
    test1.carti_points#gives the cartisian(x,y) coordinates of the test points
    test1.polar_points#gives the polar(r,theta) coordinates of the test points
```

![test points](/test_points.png)

## dock_test_auto

Script to automate testing using *Docking_OOP.py* and *gimme_mah_points.py* to dock and generate test points of different radius and angle from the tag. Results are then recorded in *data.csv*.

## Misc notes
- This code modified the original usb_cam source code to flip the camera image. Refer to the attached pdf ARUCO Detection TARS document for more info to revert.
- Failed test includes dubin path approach to the same problem but seems to perform worst than the turn-move-turn approach in both simulation and hardware
- Docking_OOP.cpp is the same as Docking_OOP.py functionally but does not seem to work due to how ROS and OOP interacts in C++. Might be fixed in the future but not now.
