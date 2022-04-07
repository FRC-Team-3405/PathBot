// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autocommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.AutoConstants;
import frc.robot.DriveConstants;
import frc.robot.RobotContainer;
import java.io.IOException;
import java.nio.file.Path;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class BlueTwoBall_Two extends CommandBase {
  RamseteCommand ramsete;
  /** Creates a new BlueTwoBall_One. */
  public BlueTwoBall_Two() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_robotDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    String trajectoryJSON = "output/BlueTwoBall2.wpilib.json";
    Trajectory Trajectory = new Trajectory();
    
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      System.out.println("Path: " + trajectoryPath);
      Trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      //testTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);

    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }

    ramsete =
        new RamseteCommand(
            Trajectory, // To use the example trajectory just replace this with exampleTrajectory.
            RobotContainer.m_robotDrive::getPose,
            new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
            new SimpleMotorFeedforward(
                DriveConstants.ksVolts,
                DriveConstants.kvVoltSecondsPerMeter,
                DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            RobotContainer.m_robotDrive::getWheelSpeeds,
            new PIDController(DriveConstants.kPDriveVel, 0.4, 0.05),
            new PIDController(DriveConstants.kPDriveVel, 0.4, 0.05),
            // RamseteCommand passes volts to the callback
            RobotContainer.m_robotDrive::tankDriveVolts,
            RobotContainer.m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    RobotContainer.m_robotDrive.resetOdometry(Trajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return;
    //return ramseteCommand.andThen(() -> RobotContainer.m_robotDrive.tankDriveVolts(0, 0)); // Does this stop the robot?
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ramsete.execute();
    RobotContainer.m_robotDrive.tankDriveVolts(0,0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_robotDrive.tankDriveVolts(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
