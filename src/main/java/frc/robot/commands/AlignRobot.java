// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.utils.Limelight;
import frc.robot.utils.Limelight.LightMode;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AlignRobot extends CommandBase {
  private boolean isFinished = false;
  private Timer t;

  /** Creates a new AlignRobot. */
  public AlignRobot() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_robotDrive);
    t = new Timer();
    // Set the neutral mode to Brake (To avoid being pushed while aligning)
    RobotContainer.m_robotDrive.frontLeft.setNeutralMode(NeutralMode.Brake); // Front Left
    RobotContainer.m_robotDrive.backLeft.setNeutralMode(NeutralMode.Brake); // Back Left
    RobotContainer.m_robotDrive.frontRight.setNeutralMode(NeutralMode.Brake); // Front Right
    RobotContainer.m_robotDrive.backRight.setNeutralMode(NeutralMode.Brake); // Back Right
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Limelight.setLedMode(LightMode.eOn);
    System.out.println("Hello, world!");
    t.start();
  }

  private boolean timerIsFinished() {
    return t.hasElapsed(5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if (!Limelight.isTarget()) {
      System.out.println("There is no target here.");
      RobotContainer.m_robotDrive.tankDriveVolts(1, -1);
      return;
    }
    

    double aX = scaleX(Limelight.getTx());
    double yaw = (aX / 2) * Constants.FOV_SUB_H;
    //System.out.println(aX);
    //System.out.println(yaw);
  
    if (Math.abs(yaw) <= 5) {
      System.out.println("Your mom says this bot is aligned!");
      RobotContainer.m_robotDrive.tankDriveVolts(0, 0);
      isFinished = true;
      return;
    }

    if (yaw > 0) {
      RobotContainer.m_robotDrive.tankDriveVolts(1, -1); // Note: 3 & -3 did not work
    } else {
      RobotContainer.m_robotDrive.tankDriveVolts(-1, 1); // Note: 3 & -3 did not work
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    isFinished = false;
    Limelight.setLedMode(LightMode.eOff);

    // Set the neutral mode to Coast (We only need Brake while aligning)
    RobotContainer.m_robotDrive.frontLeft.setNeutralMode(NeutralMode.Coast); // Front Left
    RobotContainer.m_robotDrive.backLeft.setNeutralMode(NeutralMode.Coast); // Back Left
    RobotContainer.m_robotDrive.frontRight.setNeutralMode(NeutralMode.Coast); // Front Right
    RobotContainer.m_robotDrive.backRight.setNeutralMode(NeutralMode.Coast); // Back Right
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timerIsFinished() && (!Limelight.isTarget() || isFinished);
  }

  private double scale(double coordinate, double resolution) {
    return coordinate;
    // double normalized = coordinate - (resolution / 2);
    // return normalized * (2 / resolution);
  }

  private double scaleY(double y) {
    return scale(y, Constants.HEIGHT);
  }

  private double scaleX(double x) {
    System.out.print("x: ");
    System.out.println(x);
    return scale(x, Constants.WIDTH);
  }

}

