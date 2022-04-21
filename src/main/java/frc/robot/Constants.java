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
    //** PDP */
    public static final int POWER_DISTRO_ID = 1;
    public static final int PCM_PORT = 0;

    //** Controllers */
    // AIRFLO Controller (Xbox Controller)
    public static final int XBOX = 0; // AIRFLO/XBOX Controller Port
    public static final int XBOX_XAXIS = 1; // arcadeDrive
    public static final int XBOX_YAXIS = 3; // arcadeDrive; Change to 4 if using an XboxController
    public static final int SHIFT_HIGHGEAR_BUTTON = 6; // Right Trigger on AirFlo
    public static final int SHIFT_LOWGEAR_BUTTON = 5; // Left Trigger on AirFlo
    public static final int ALIGN_ROBOT_BUTTON = 8;

    // Joystick Controller (Secondary Driver)
    public static final int JOYSTICK = 1; // Joystick Controller Port
    public static final int INTAKE_POSITION_PISTON = 2; // Piston to bring intake arm in and out
    public static final int SHOOT_BALL = 4; // Run the motors to shoot the ball!
    public static final int INTAKE_BUTTON = 5;
    public static final int SHOOT_BUTTON = 6;

    //** DriveTrain */
    //public static final double DRIVE_ENCODER_RESOLUTION = 8192.0; // Rev's ThroughBore Encoders read 8192 ticks per rotation.
    public static final double DRIVE_ENCODER_RESOLUTION = 2048.0; // Rev's ThroughBore Encoders read 8192 ticks per rotation.
    public static final double DRIVE_WHEEL_DIAMETER = 0.1524; // 6 Inches in meters is 0.1524
    public static final double DISTANCE_PER_PULSE = (DRIVE_WHEEL_DIAMETER * Math.PI) / DRIVE_ENCODER_RESOLUTION;
    public static final double MAX_POWER = 0.8; // Max Power to the motors during Teleop mode
    public static final double MAX_TURN_POWER = 0.8; // Max Power to the motors while turning during Teleop mode
    public static final int COMPRESSOR_PORT = 0; // Compressor
    public static final int HIGHGEAR = 2; // Double Solenoid kForward port
    public static final int LOWGEAR = 3; // Double solenoid kReverse port
    

    //** LimeLight Camera */
    public static final double KNOWN_DISTANCE = 3.048; // 10 Feet (120 inches in meters)
    public static final int PIXEL_WIDTH_KNOWN = 60;
    public static final double KNOWN_TAPE_BOUND_WIDTH = 0.3937; // Width of the tape (15.5 inches converted to meters)
    public static final double FOCAL_LENGTH = (KNOWN_DISTANCE * PIXEL_WIDTH_KNOWN) / KNOWN_TAPE_BOUND_WIDTH; //** This is an equation of some kind pulled from this repo: https://github.com/Yeti-Robotics/Y3T1-java-2021/blob/master/src/main/java/frc/robot/RobotContainer.java */
    public static final int WIDTH = 960;
    public static final int HEIGHT = 720;
    public static final double FOV_SUB_H = 59.6;
    public static final double FOV_SUB_V = 45.7;

    //** Shooter */
    public static final double SHOOTER_HEIGHT = 0.635; //(25 inches)
    public static final double GRAVITY = 9.81; // Meters/sec^2

    //** Intake System */
    public static final int EXTENDER_FORWARD = 2;
    public static final int EXTENDER_BACKWARDS = 3;
    public static final int INTAKE_VICTOR = 10;
    public static final int INTAKE_IN = 6;
    public static final int INTAKE_OUT = 7;
    

    //** Ball Processing System */
    public static final int TOWER_FEED_MOTOR = 5; // Tower
    //public static final int BREAKLIGHT_PORT = 0; // Breakbeam sensor

    //public static final int BREAK_LIGHT = 2;
    public static final int BACK_FEED = 3;

    public static final int SHOOTER_MOTOR = 20;

    //** Climber System */
    public static final int CLIMBER_MOTOR_1 = 11;
    public static final int CLIMBER_MOTOR_2 = 12;
    public static final int SHOOTER_MOTOR_TWO = 21;
    public static final int ORANGE_RIGHT = 52;
    public static final int ORANGE_LEFT = 53;
    public static final int LOWER_TOWER_MOTOR = 5;
    public static final int UPPER_TOWER_MOTOR = 6;
    public static final int STRAY_TOWER_MOTOR = 7;
    
}
