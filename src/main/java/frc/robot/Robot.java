// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.opencv.core.Mat;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DriveTrain;
//import frc.robot.subsystems.DriveTrain.ShifterStatus;
import frc.robot.utils.Limelight;
import frc.robot.utils.Limelight.LightMode;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  Thread m_visionThread;
  public static RobotContainer m_robotContainer;
  public static NetworkTable networkTable; // LimeLight Network Tables
  public static NetworkTable rootNetworkTable; // LimeLight Network Tables

  DriveTrain DriveTrain;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();

    // LimeLight Network Tables
    networkTable = NetworkTableInstance.getDefault().getTable("SmartDashboard");
    rootNetworkTable = NetworkTableInstance.getDefault().getTable("");

    m_visionThread = new Thread(
        () -> {
          // Get the UsbCamera from CameraServer
          UsbCamera camera = CameraServer.startAutomaticCapture();
          // Set the resolution
          camera.setResolution(640, 480);

          // Get a CvSink. This will capture Mats from the camera
          CvSink cvSink = CameraServer.getVideo();
          // Setup a CvSource. This will send images back to the Dashboard
          CvSource outputStream = CameraServer.putVideo("Rectangle", 640, 480);

          // Mats are very memory expensive. Lets reuse this Mat.
          Mat mat = new Mat();

          // This cannot be 'true'. The program will never exit if it is. This
          // lets the robot stop this thread when restarting robot code or
          // deploying.
          while (!Thread.interrupted()) {
            // Tell the CvSink to grab a frame from the camera and put it
            // in the source mat. If there is an error notify the output.
            if (cvSink.grabFrame(mat) == 0) {
              // Send the output the error.
              outputStream.notifyError(cvSink.getError());
              // skip the rest of the current iteration
              continue;
            }
            // Put a rectangle on the image
            //Imgproc.rectangle(
            //   mat, new Point(100, 100), new Point(400, 400), new Scalar(255, 255, 255), 5);
            // Give the output stream a new image to display
            outputStream.putFrame(mat);
          }
        });
    m_visionThread.setDaemon(true);
    m_visionThread.start();  
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */

  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.

    CommandScheduler.getInstance().run();
    // LimeLight Network Tables
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry tlong = table.getEntry("tlong");

    // Read values from the NetworkTables for the LimeLight
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
    double schlong = tlong.getDouble(0.0);

    // Post LimeLight Values to the SmartDashboard
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);

    SmartDashboard.putNumber("schlong", schlong);

    SmartDashboard.putNumber("distance", Limelight.getDistance());

    SmartDashboard.putNumber("hor distance", Limelight.getHorDistance());

    // // Post different values to troubleshoot the bot from a distance
    // if (frc.robot.subsystems.DriveTrain.getShifterGear() == ShifterStatus.HIGH) {
    //   SmartDashboard.putString("Shifter Gear", "HIGH");
    // } // Shifter Status (Set to HIGH)
    // else if (frc.robot.subsystems.DriveTrain.getShifterGear() == ShifterStatus.LOW) {
    //   SmartDashboard.putString("Shifter Gear", "LOW");
    // } // Shifter Status (Set to LOW)
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    // ** I'm not sure why this value is never getting put on the dashboard. Perhaps
    // I need to call a shifter button to have it publish? */
    // Post different values to troubleshoot the bot from a distance
    // if (frc.robot.subsystems.DriveTrain.getShifterGear() == ShifterStatus.HIGH) {
    //   SmartDashboard.putString("Shifter Gear", "HIGH");
    // } // Shifter Status (Set to HIGH)
    // else if (frc.robot.subsystems.DriveTrain.getShifterGear() == ShifterStatus.LOW) {
    //   SmartDashboard.putString("Shifter Gear", "LOW");
    // } // Shifter Status (Set to LOW)
  }

  @Override
  public void disabledPeriodic() {
    Limelight.setLedMode(LightMode.eOff); // Turn the LimeLight off when disabled but powered on
  }


  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    Limelight.setLedMode(LightMode.eOn); // Turn the LimeLight on during test mode to tune the feed
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
