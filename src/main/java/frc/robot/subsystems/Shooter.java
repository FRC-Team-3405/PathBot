package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    WPI_TalonFX shooter = new WPI_TalonFX(Constants.SHOOTER_MOTOR);

    public void shoot(double speed) {
        shooter.set(ControlMode.PercentOutput, speed);
    }
}
