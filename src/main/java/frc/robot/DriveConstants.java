package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

public final class DriveConstants {

    public static int FR_TALONFX = 1;
    public static int BR_TALONFX = 2;
    public static int FL_TALONFX = 3;
    public static int BL_TALONFX = 4;
    public static int[] kLeftEncoderPorts = {2,3};
    public static boolean kLeftEncoderReversed = false;
    public static int[] kRightEncoderPorts = {0,1};
    public static boolean kRightEncoderReversed = true;
    public static double kEncoderDistancePerPulse = 1/8192 * 6 * Math.PI; // 1 over cycles/pulses per rotation times circumfrence of the wheel (6 * pi)
    //public static double kEncoderDistancePerPulse = 1/2048 * 6 * Math.PI; // 1 over cycles/pulses per rotation times circumfrence of the wheel (6 * pi)

    public static double ksVolts = 0.57919; // Speed
   // public static double kvVoltSecondsPerMeter = 16.222; // Velocity
    public static double kvVoltSecondsPerMeter = 2.111; // Velocity

    public static double kaVoltSecondsSquaredPerMeter = 0.44883; // Acceleration
    public static DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(0.65836);
    public static double kPDriveVel = 3.7065E-09;
    
}
