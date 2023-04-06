# Aue Finals

# Gazebo Simulation

## Task 1

The wall following and obstacle avoidance functionality of the robot use the same algorithm. This algorithm follows the following steps:

1. Divide the lidar into two left and right sections, each of 60 degrees.
2. The lidar outputs values between 0 and 3.5 with anything greater than 3.5 corresponding to infinity. Normalize the lidar data to be between 0 and 1 using the sigmoid function. Set any infinite values to 1. For the PID controller, we want objects that are closer to the turtlebot to provide larger numbers, so complete the normalization process by subtracting each normalized lidar output from 1. Now we have lidar data in the bounds [0, 1] with values closer to 1 representing objects that are closer to the turtlebot and those closer to 0 representing values further away.
3. Take a sum of the normalized lidar values for the left and rigth sections. Find the difference between them. This difference becomes the error for our PID controller. Input this error into our PID controller to get angular velocity for the turtlebot.

The parameters for the PID controller had to be precisely tuned to get the desired behavior for the turtlebot.

### Wall Following

![alt text](https://github.com/gbbyrd/Aue823_Spring22_Team1/blob/master/catkin_ws/src/auefinals/src/videos/wall_following.gif)

### Obstacle Avoidance

![alt text](https://github.com/gbbyrd/Aue823_Spring22_Team1/blob/master/catkin_ws/src/auefinals/src/videos/obstacle_avoidance.gif)

## Task 2

### Line Following

The line following algorithm uses blob detection and masking. We started by tuning the hsv parameters to get a binary image of the turtlebot camera feed that included only the line that we need to follow. We then masked this binary image by cropping out all but the very bottom of the image (the part of the line closest to the turtlebot). We then calculate the centroid of the binary image, which corresponds to the center of the line. Next, we treat the difference between the x position of the centroid and the center of the image as our error for a tuned PID controller which turns the bot to have it follow the line successfully. This centroid of the line is denoted in red in the below .gif.

### Stop Sign Detection

The implementation of stop sign detection was done using a pretrained Yolov5 model (a neural network for object detection). The smallest version of this model was chosen for performance purposes, but it is more than sufficient for the application here. In fact, our Yolo model is capable of detecting and labelling approximately 80 objects! The stop sign detection works by constantly scanning the camera feed using yolo and providing a a pandas dataframe consisting of the name of any classes of objects detected in the picture as well as the bounding box dimenstions of the detected object. We loop through this dataframe searching for the stop sign class, if the stop sign class is found in the dataframe, then a stop sign is detected. We only want to stop if the stop sign is close enough to be relevant, so a threshold is established and compared to the size of the bounding block. If the size of the bounding block is greater than this threshold, then the turtlebot stops for 3 seconds before continueing on.

![alt text](https://github.com/gbbyrd/Aue823_Spring22_Team1/blob/master/catkin_ws/src/auefinals/src/videos/stop_sign_detection.gif)

# Initial README from TAs
## AuE893 Final Project Gazebo Model

Dependencies that are not included:

* The TB3 packages

To bring up the Gazebo Model:

roslaunch aue_finals turtlebot3_autonomy_finals.launch


There are 3 sections to this project:

* Task 1: Wall following/Obstacle avoidance - The Turtlebot starts here. It must successfully follow the wall and avoid the obstacles until it reaches the yellow line.
* Task 2:
- Line following - The Turtlebot must successfully follow the yellow line.
- Stop Sign detection - While navigating the yellow line, the the Turtlebot should stop at the stop sign for 3 seconds before continuing. The stop-sign will be detected by TinyYOLO.
* Task 3: AprilTag tracking - Spawn another TB3 in the environment past the line. Attach an AprilTag onto the robot. You must teleop this TB3 (there are several packages available online, to enable teleop terminals; use the namespace concept to send seperate /cmd_vel values to seperate robots), and the preceding TB3 should follow it.


This model contains code from the following repositories:

* (TB3 model + inpsiration) TB3 Autorace: https://github.com/ROBOTIS-GIT/turtlebot3_autorace_2020.git

Maintainers:

* Dhruv Mehta (TA)
* Sumedh Sathe (TA)
