# Team 2 - Assignment 4

## PART 1

### Gazebo line follower

`roslaunch assignment4_trackingandfollowing turtlebot_follow_line.launch`

To successfully follow the line, a binary image is first created that outputs all
pixels of the path as 1 and all other pixels as zero. We then split the camera image
into right and left images and sum up the pixel values of both sides. The ratio of 
right pixels to left pixels is used as the error for our PID controller.

A new an improved algorithm that uses the same method as the real line following below was implemented in gazebo. This video can also be seen in the videos folder and shows a turtlebot following the line at a very high speed.

![alt text](https://github.com/gbbyrd/Aue823_Spring22_Team1/blob/master/catkin_ws/src/assignment4_trackingandfollowing/line_follower_gazebo.png?raw=true)

### Real world line follower

In order to run the real world line follower, you must first run the bringup and
the raspi camera launch commands on the physical turtlebot. To run the script, use
the below command:

`rosrun assignment4_trackingandfollowing real_line_follower.py`

To successfully follow the line on the real bot a different approach was taken. A
binary image was created like in the gazebo line follower, but this time we masked
the image and found the centroid of the detected portion of the line. The distance
between this centroid and the center of our image is used as the error for our
PID controller.

![alt text](https://github.com/gbbyrd/Aue823_Spring22_Team1/blob/master/catkin_ws/src/assignment4_trackingandfollowing/line_follower_real.png?raw=true)

## PART 2

### April Tag Detection and Following

![alt text](https://github.com/Parth-S-Hub/Aue823_Spring22_Team1/blob/master/catkin_ws/src/assignment4_trackingandfollowing/src/video/apriltag_detection_AdobeExpress%20(1).gif)

This works on real world bot as it detects and follows an april tag. You must first 
run bringup and also the camera launch command on the physical turtlebot. To run the script,
use the below command:

`rosrun assignment4_trackingandfollowing apriltag_tracking.py`

It is recommended to have a stable connection between the bot and operating PC as 
this may affect the transmission speed of the image. One can also use compressed images
to transfer from turtlebot to PC to solve the error.

The tutlebot detects the april tag and then subsequently the pose of the tag. Based on the
relative pose estimate, the turtlebot follows the tag.

#### April tag detection and Following (Camera-View)
![alt text](https://github.com/Parth-S-Hub/Aue823_Spring22_Team1/blob/master/catkin_ws/src/assignment4_trackingandfollowing/src/video/apriltag_follow_AdobeExpress.gif)

#### April tag detection and Following

![alt text](https://github.com/Parth-S-Hub/Aue823_Spring22_Team1/blob/master/catkin_ws/src/assignment4_trackingandfollowing/src/video/VID-20230328-WA0010_AdobeExpress.gif)


