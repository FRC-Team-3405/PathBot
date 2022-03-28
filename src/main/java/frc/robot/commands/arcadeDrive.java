// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants; // Constants File
import frc.robot.RobotContainer; // RobotContainer File

public class arcadeDrive extends CommandBase {
  /** Creates a new ArcadeDrive. */
  public arcadeDrive() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_robotDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double moveSpeed = -RobotContainer.xbox.getRawAxis(Constants.XBOX_XAXIS)*Constants.MAX_POWER; // Limit the moveSpeed to 80% Power
    double rotateSpeed = RobotContainer.xbox.getRawAxis(Constants.XBOX_YAXIS)*Constants.MAX_TURN_POWER; // Limit the rotateSpeed to 80% Power
    // RobotContainer.drivetrain.arcadeDrive(moveSpeed, rotateSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // RobotContainer.drivetrain.arcadeDrive(0,0); // Stops the motors when the robot is disabled or enters teleop mode AFTER auto
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}