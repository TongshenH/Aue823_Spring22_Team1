#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt, pi



class TurtleBot:

    def __init__(self):
        # Creates a node with name 'turtlebot_controller' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node('turtlebot_controller', anonymous=True)

        # Publisher which will publish to the topic '/turtle1/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel',
                                                  Twist, queue_size=10)

        # A subscriber to the topic '/turtle1/pose'. self.update_pose is called
        # when a message of type Pose is received.
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose',
                                                Pose, self.update_pose)

        self.pose = Pose()
        self.rate = rospy.Rate(10)

    def update_pose(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)

    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose.x - self.pose.x), 2) +
                    pow((goal_pose.y - self.pose.y), 2))

    def linear_vel(self, goal_pose, constant=0.5):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def angular_vel(self, goal_pose, constant=5):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        angle1 = self.steering_angle(goal_pose) + pi * 2 - self.pose.theta
        angle2 = self.steering_angle(goal_pose) - pi * 2 - self.pose.theta
        angle3 = self.steering_angle(goal_pose) - self.pose.theta
        angle = min([angle1, angle2, angle3], key=abs)
        return constant * angle

    def angular_control(self, goal_pose, constant=3):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        angle1 = goal_pose.theta + pi * 2 - self.pose.theta
        angle2 = goal_pose.theta - pi * 2 - self.pose.theta
        angle3 = goal_pose.theta - self.pose.theta
        angle = min([angle1, angle2, angle3], key=abs)
        return constant * angle

    def move2goal(self, position):
        """Moves the turtle to the goal."""
        goal_pose = Pose()

        # # Get the coordinate
        goal_pose.x = position[0]
        goal_pose.y = position[1]

        # Set the distance tolerance
        distance_tolerance = rospy.get_param('~tol')

        vel_msg = Twist()

        while self.euclidean_distance(goal_pose) > distance_tolerance:
            # Linear velocity in the x-axis.
            vel_msg.linear.x = self.linear_vel(goal_pose)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel(goal_pose)
            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)
            # Publish at the desired rate.
            self.rate.sleep()

        # Stopping our robot after the movement is over.
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
        # If we press control + C, the node will stop.
        # rospy.spin()

    def move2goal_straight(self, step):
        """Moves the turtle to the goal."""
        goal_pose = Pose()

        # Get the coordinate info
        goal_pose.x = step[0]
        goal_pose.y = step[1]
        goal_pose.theta = step[2]

        # Get the distance tolerance and angle tolerance
        distance_tolerance = rospy.get_param('~tol')
        angle_tolerance = rospy.get_param('~atol')
        vel_msg = Twist()

        while self.euclidean_distance(goal_pose) > distance_tolerance:
            # Linear velocity in the x-axis.
            vel_msg.linear.x = self.linear_vel(goal_pose)
            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)
            # Publish at the desired rate.
            self.rate.sleep()

        while abs(self.pose.theta - goal_pose.theta) > angle_tolerance:
            vel_msg.angular.z = self.angular_control(goal_pose)
            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)
            # Publish at the desired rate.
            self.rate.sleep()

        # Initialize the angle speed
        vel_msg.angular.z = 0
        vel_msg.linear.x = 0
        self.velocity_publisher.publish(vel_msg)


    def circle(self):
        """Moves the turtle to the goal."""
        goal_pose = Pose()

        # Get the input from the user.
        # goal_pose.x = float(input("Set your x goal: "))
        goal_pose.x = rospy.get_param('~x')
        # goal_pose.y = float(input("Set your y goal: "))
        goal_pose.y = rospy.get_param('~y')

        # Please, insert a number slightly greater than 0 (e.g. 0.01).
        # distance_tolerance = float(input("Set your tolerance: "))
        distance_tolerance = rospy.get_param('~tol')

        vel_msg = Twist()

        while self.euclidean_distance(goal_pose) > distance_tolerance:
            # Porportional controller.
            # https://en.wikipedia.org/wiki/Proportional_control

            # Linear velocity in the x-axis and y-axis.
            vel_msg.linear.x = 8
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 8

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)

            # Publish at the desired rate.
            self.rate.sleep()

        # Stopping our robot after the movement is over.
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

        # If we press control + C, the node will stop.
        rospy.spin()


    def square_openloop(self):

        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        vel_msg = Twist()

        while not rospy.is_shutdown():
            # Set the current distance
            distance = 4
            angle = pi / 2

            for steps in range(4):
                t0 = rospy.Time.now().to_sec()
                current_distance1 = 0
                # Initialize the zero angular speed
                vel_msg.angular.z = 0
                while (current_distance1 <= distance):
                    # Publish the velocity
                    self.velocity_publisher.publish(vel_msg)
                    # Set the x-axis speed
                    vel_msg.linear.x = 1.2
                    # Take actual time to velocity calculus
                    t1 = rospy.Time.now().to_sec()
                    t_total1 = t1 - t0
                    current_distance1 = vel_msg.linear.x * t_total1
                t0 = rospy.Time.now().to_sec()
                current_angle1 = 0
                # Initialize the zero longitudinal speed
                vel_msg.linear.x = 0
                while (current_angle1 <= angle):
                    # Publish the angular velocity
                    self.velocity_publisher.publish(vel_msg)
                    # Set the angular speed
                    vel_msg.angular.z = 1.2
                    t1 = rospy.Time.now().to_sec()
                    t_total2 = t1 - t0
                    current_angle1 = t_total2 * vel_msg.angular.z

            # Stopping our robot after the movement is over.
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
            self.velocity_publisher.publish(vel_msg)

            # If we press control + C, the node will stop.
            rospy.spin()


    def square_closedloop(self):
        """Moves the turtle to the goal."""
        # Set the world coordinate
        first_coordinate = [5, 5]
        self.move2goal(first_coordinate)
        # Path planning
        coordinate = [[5, 5, 0], [8, 5, pi/2], [8, 8, pi], [5, 8, -pi/2], [5, 5, 0]]
        for step in coordinate:
            self.move2goal_straight(step)
        rospy.spin()



if __name__ == '__main__':
    try:
        x = TurtleBot()
        # x.circle()
        x.square_openloop()
        # x.square_closedloop()
    except rospy.ROSInterruptException:
        pass
