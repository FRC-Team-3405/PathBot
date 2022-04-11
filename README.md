# Andre the Giant - Rapid React 2022

### Main Driver Controller Map (AIRFLO Controller)

The robot is driven using a Differential Drive (```arcadeDrive```). 

- ```Left Joystick```: Move forward / Backwards
- ```Right Joystick```: Move left / right
- ```Left Bumper```: Shift High
- ```Right Bumper```: Shift Low

### Secondary Driver Controller Map (Xbox Controller)

- ```A Button```: Move climber arm down
- ```Y Button```: Move climer arm up
- ```Right Bumper```: Spin the shooter Motors
- ```Right Trigger```: Move the tower motors (feed the shooter)

### Good Notes
A special thanks goes to team 3374 (the JacksonHole RoboBroncs) and their programmer Ben for suggesting a current limit on the CTRE Falcon 500 motors. Not doing this will cause the motors to spin until an internal breaker flips, causing the robot unmovable for the remainder of the match. These have been implemented in the ```DriveTrain``` subsystem.
