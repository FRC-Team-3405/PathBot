// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  public Climber() {
    // Rev NEO Brushless Motors
    CANSparkMax climbMotor1 = new CANSparkMax(Constants.CLIMBER_MOTOR_1, CANSparkMax.MotorType.kBrushless);
    CANSparkMax climbMotor2 = new CANSparkMax(Constants.CLIMBER_MOTOR_2, CANSparkMax.MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
