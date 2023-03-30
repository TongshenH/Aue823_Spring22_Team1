# Team 2

## PART 1

### Gazebo line follower

`roslaunch assignment4_trackingandfollowing turtlebot_follow_line.launch`

To successfully follow the line, a binary image is first created that outputs all
pixels of the path as 1 and all other pixels as zero. We then split the camera image
into right and left images and sum up the pixel values of both sides. The ratio of 
right pixels to left pixels is used as the error for our PID controller.

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

