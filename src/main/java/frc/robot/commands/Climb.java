// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

public class Climb extends CommandBase {
  /** Creates a new Shooter. */
  public Climb() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(RobotContainer.xbox2.getAButton());
    if (RobotContainer.xbox2.getAButton()) {
      RobotContainer.m_climber.setSpeed(-.1);
    } else {
      RobotContainer.m_climber.setSpeed(0);
    }
    
    if (RobotContainer.xbox2.getYButton()) {
      RobotContainer.m_climber.setSpeed(.1);
    } else {
      RobotContainer.m_climber.setSpeed(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // RobotContainer.m_shooter.tower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}