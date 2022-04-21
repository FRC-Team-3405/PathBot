# Andre the Giant - Rapid React 2022
Andre the Giant was a French wrestler and actor. He was known for his great size, which led to him being called the "Eighth Wonder of the World". He was most known globally for his role in the hit Disney film _The Princess Bride_, in which he played the beloved rhyming character Fezzik. Packing great strength into an endless heart, Andre (and Fezzik) is a reliable, unstoppable human being. 

Many of his characteristics are found in this year's iteration of Team 3405's competitive robot, except for one. Our robot, fondly named after Andre, packs great defensive strength into a small size. The bot is virtually unstoppable and will never be pushed around due to its West-Coast drivebase powered by four CTRE Falcon500 motors and a two-speed gearbox. Vision processing is achieved through the exclusive use of a single LimeLight camera, with a Microsoft USB camera being used for Driver assistance rather than actual processing. Someday, our version of Andre the Giant may be able to find rhymes to everything, but we do not have a robot named Vicinni to annoy with constant rhymes. Therefore, this may never actually happen. Previous versions of the robot have included names such as "The Robot of Unusual Size", the "Inconceivable Bot", and "Inigo Montoya".



## Robot at a Glance

### Main Driver Controller Map (AIRFLO Controller)

The robot is driven using a Differential Drive (```arcadeDrive```). 

- ```Left Joystick```: Move forward / Backwards
- ```Right Joystick```: Move left / right
- ```Right Bumper```: Toggle Gear Shift; Low Gear by Default
- ```Right Trigger```: "Align Robot" Command. Requires the LimeLight camera and DriveTrain to be operational.

### Secondary Driver Controller Map (Xbox Controller)

- ```A Button```: Move climber arm down
- ```B Button```: Extend the Intake Arm
- ```X Button```: Retract the Intake Arm
- ```Y Button```: Move climber arm up
- ```Right Bumper```: Spin the shooter Motors
- ```Right Trigger```: Move the tower motors (feed the shooter)

### Notes
A special thanks goes to team 3374 (the JacksonHole RoboBroncs) and their programmer Ben for suggesting a current limit on the CTRE Falcon 500 motors. Not doing this will cause the motors to spin until an internal breaker flips, rendering the robot inoperable for the remainder of the match. These have been implemented in the ```DriveTrain``` subsystem.



## Technical Information
### DriveTrain
- Four ***CTRE Falcon500*** motors: Arguably the best motors in all of FRC History and our "secret" ingredient. These will never stop for ***anything***!
- ***Two-Speed*** Gearbox: For when the robot we're defending decides it wants to push back: We can push anyone and anything around!
- ***Efficient and Power-Aware***: Current limits (and other spices) to prevent browning and provide infinite robot-pushing power.
