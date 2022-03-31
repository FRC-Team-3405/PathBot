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
    public static double kEncoderDistancePerPulse = 1/8192 * .1524 * Math.PI; // 1 over cycles/pulses per rotation times circumfrence of the wheel (6 * pi)
    //public static double kEncoderDistancePerPulse = 1/2048 * .1524 * Math.PI; // 1 over cycles/pulses per rotation times circumfrence of the wheel (6 * pi)

    public static double ksVolts = 0.53732; // Speed
   // public static double kvVoltSecondsPerMeter = 16.222; // Velocity
    //public static double kvVoltSecondsPerMeter = 16.023; // Velocity
    //public static double kvVoltSecondsPerMeter = 3.111; // Velocity
    public static double kvVoltSecondsPerMeter = 5.5; // Velocity

    public static double kaVoltSecondsSquaredPerMeter = 0.44883; // Acceleration
    //public static double kaVoltSecondsSquaredPerMeter = 2.0937; // Acceleration

    public static DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(0.65836);
    public static double kPDriveVel = 3.7065E-09; // ORIGINAL
    //public static double kPDriveVel = 21.326;
    //public static double kPDriveVel = 4; // Tuning PID Controller
    
}
