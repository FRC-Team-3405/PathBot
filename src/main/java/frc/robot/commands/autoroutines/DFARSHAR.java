// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoroutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.*;
import frc.robot.commands.autocommands.SpinShooterMotor;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DFARSHAR extends SequentialCommandGroup {
  /** Creates a new DFARSHAR (Drive Forward - Align Robot - Shoot - Auto Routine). */
  public DFARSHAR() {
    addCommands(
      new DriveForward(), // Drive forward for 1 second
      new AlignRobot(), // Align the robot with the goal
      new SpinShooterMotor() // Shoot the preloaded ball
    );
  }
}
