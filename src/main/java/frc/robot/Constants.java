// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    //** Controllers */
    // AIRFLO Controller (Xbox Controller)
    public static final int XBOX = 0; // AIRFLO/XBOX Controller Port
    public static final int XBOX_XAXIS = 1; // arcadeDrive
    public static final int XBOX_YAXIS = 4; // arcadeDrive; Change to 4 if using an XboxController
    public static final int SHIFT_HIGHGEAR_BUTTON = 8; // Right Trigger on AirFlo
    public static final int SHIFT_LOWGEAR_BUTTON = 7; // Left Trigger on AirFlo

    // Joystick Controller (Secondary Driver)
    public static final int JOYSTICK = 1; // Joystick Controller Port
    public static final int INTAKE_MOTOR_RUN = 1; // Run the intake motors to pick up balls
    public static final int INTAKE_POSITION_PISTON = 2; // Piston to bring intake arm in and out
    public static final int READ_BALL_COLOR = 3; // Get the color sensor's reading, alert the user if the ball is red or blue.
    public static final int SHOOT_BALL = 4; // Run the motors to shoot the ball!

    //** DriveTrain */
    //public static final double DRIVE_ENCODER_RESOLUTION = 8192.0; // Rev's ThroughBore Encoders read 8192 ticks per rotation.
    public static final double DRIVE_ENCODER_RESOLUTION = 2048.0; // Rev's ThroughBore Encoders read 8192 ticks per rotation.

    public static final double DRIVE_WHEEL_DIAMETER = 0.1524; // 6 Inches in meters is 0.1524
    public static final double DISTANCE_PER_PULSE = (DRIVE_WHEEL_DIAMETER * Math.PI) / DRIVE_ENCODER_RESOLUTION;
    public static final double MAX_POWER = 0.8; // Max Power to the motors during Teleop mode
    public static final double MAX_TURN_POWER = 0.8; // Max Power to the motors while turning during Teleop mode
    public static final int COMPRESSOR_PORT = 0; // Compressor
    public static final int HIGHGEAR = 1; // Double Solenoid kForward port
    public static final int LOWGEAR = 2; // Double solenoid kReverse port
    

    //** LimeLight Camera */
    public static final int KNOWN_TAPE_BOUND_WIDTH = 5; // Width of the tape
    public static final int FOCAL_LENGTH = 0; //** This is an equation of some kind pulled from this repo: https://github.com/Yeti-Robotics/Y3T1-java-2021/blob/master/src/main/java/frc/robot/RobotContainer.java*/

    public static final int EXTENDER_FORWARD = 2;
    public static final int EXTENDER_BACKWARDS = 3;
    public static final int INTAKE_VICTOR = 4;

    public static final int COLOR_SENSOR = 1;
    public static final int BREAK_LIGHT = 2;
    
    
}
