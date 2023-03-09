# Team1 Assignment 2B


## Type the following in the terminal to run the bot in a circle:


$ roslaunch assignment2B_turtlebot3 move.launch code:=circle 


## Parameters considered for slow, medium & fast speed:

### Slow Speed:
Linear Velocity: 0.1, Angular Velocity: 0.1

### Medium Speed:
Linear Velocity: 0.6, Angular Velocity: 0.6

### Fast Speed:
Linear Velocity: 1.2, Angular Velocity: 1.2


## Remarks:
As the provided twist vel ( velocity )  increases, the bot covers a shorter circular path and at higher velocities the bot becomes unstable.

---

## Type the following in terminal to run the bot in a open loop square :


$ roslaunch assignment2B_turtlebot3 move.launch code:=square


## Parameters considered for slow, medium & fast speed:

### Slow Speed:
Linear Velocity: 0.3, Angular Velocity: 0.3

### Medium Speed:
Linear Velocity: 0.6, Angular Velocity: 0.6

### Fast Speed:
Linear Velocity: 1.2, Angular Velocity: 1.2


## Remarks:
As the provided twist vel ( velocity ) increases, since the square tracking is an open loop based on time to reach a particular distance or to rotate a certain agle; as the speed increases the bot deviates from the set distance to track ( here, 1m x 1m square ) and also increasing the angular velocity causes the bol to deviate form the angle of a square (90).  


