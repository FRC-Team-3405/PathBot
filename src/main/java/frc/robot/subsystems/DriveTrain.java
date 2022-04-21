package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
// import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
// import edu.wpi.first.wpilibj.PneumaticsControlModule;
// import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.DriveConstants;

public class DriveTrain extends SubsystemBase {
  // Shifter Status
  // public enum ShifterStatus{
  //   HIGH, LOW
  //}
  // public static ShifterStatus shifterStatus;
  // Compressor
   // Instantiate a Compressor
  public static DoubleSolenoid m_shift = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.HIGHGEAR, Constants.LOWGEAR);
  public static boolean LowGear = false;

  private final WPI_TalonFX frontLeft = new WPI_TalonFX(DriveConstants.FL_TALONFX);
  private final WPI_TalonFX backLeft = new WPI_TalonFX(DriveConstants.BL_TALONFX);
  private final WPI_TalonFX frontRight = new WPI_TalonFX(DriveConstants.FR_TALONFX);
  private final WPI_TalonFX backRight = new WPI_TalonFX(DriveConstants.BR_TALONFX);
  

  // The motors on the left side of the drive.
  // The motors on the right side of the drive.
  private final MotorControllerGroup m_leftMotors =
      new MotorControllerGroup(
          frontLeft,
          backLeft);

  // The motors on the right side of the drive.
  private final MotorControllerGroup m_rightMotors =
      new MotorControllerGroup(
          frontRight,
          backRight);

  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  // The left-side drive encoder
  private final Encoder m_leftEncoder =
      new Encoder(
          DriveConstants.kLeftEncoderPorts[0],
          DriveConstants.kLeftEncoderPorts[1],
          DriveConstants.kLeftEncoderReversed);

  // The right-side drive encoder
  private final Encoder m_rightEncoder =
      new Encoder(
          DriveConstants.kRightEncoderPorts[0],
          DriveConstants.kRightEncoderPorts[1],
          DriveConstants.kRightEncoderReversed);

  // The gyro sensor
  private final Gyro m_gyro = new ADXRS450_Gyro();

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  void setFalconLimit(WPI_TalonFX talon) {
    talon.configSupplyCurrentLimit(
      new SupplyCurrentLimitConfiguration(true, 39, 40, 10)
    );
    talon.configStatorCurrentLimit(
      new StatorCurrentLimitConfiguration(true, 70, 90, 1.0)
    );
  }

  /** Creates a new DriveSubsystem. */
  public DriveTrain() {
    setFalconLimit(frontRight);
    setFalconLimit(backRight);
    setFalconLimit(frontLeft);
    setFalconLimit(backLeft);
    m_shift.set(Value.kForward);

    // Shift Gears
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotors.setInverted(true);

    // Sets the distance per pulse for the encoders
    m_leftEncoder.setDistancePerPulse(Constants.DISTANCE_PER_PULSE);
    m_rightEncoder.setDistancePerPulse(Constants.DISTANCE_PER_PULSE);

    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
  }
  
  // Shift to High Gear, Update the SmartDashboard to show we are in HIGH gear
  public void shiftHigh(){
    m_shift.set(Value.kForward);
    System.out.println(m_shift);
    //shifterStatus = ShifterStatus.HIGH;
  }

  // Shift to Low Gear, Update the SmartDashboard to show we are in LOW gear
  public void shiftLow(){
    m_shift.set(Value.kReverse);
    //shifterStatus = ShifterStatus.LOW;
  }

  public void shift() {
    m_shift.toggle();
    LowGear = !LowGear;
    System.out.println("Low Gear");
  }

  // Test Stuff
  NetworkTableEntry m_xEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("X");
  NetworkTableEntry m_yEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Y");
  
  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        //m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance()); // Multiply these values by the DISTANCE_PER_PULSE constant?
        m_gyro.getRotation2d(), m_leftEncoder.getDistance()*Constants.DISTANCE_PER_PULSE, m_rightEncoder.getDistance()*Constants.DISTANCE_PER_PULSE); // Testing stuff    edit: IT WORKS DO NOT DELETE
    m_odometry.update(Rotation2d.fromDegrees(getHeading()), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());

    var translation = m_odometry.getPoseMeters().getTranslation();
    m_xEntry.setNumber(translation.getX());
    m_yEntry.setNumber(translation.getY());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }


  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(rightVolts);
    m_drive.feed();
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }


  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public double getLeftEncoder() {
    return (m_leftEncoder.get() * Constants.DISTANCE_PER_PULSE);
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public double getRightEncoder() {
    return (m_rightEncoder.get() * Constants.DISTANCE_PER_PULSE);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (getLeftEncoder() + getRightEncoder()) / 2;
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_gyro.getRate();
  }
  // public static ShifterStatus getShifterGear() {
  //   return shifterStatus;
  // }
}