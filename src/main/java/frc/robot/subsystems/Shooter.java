package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    WPI_TalonFX rightShooter = new WPI_TalonFX(Constants.SHOOTER_MOTOR);
    WPI_TalonFX leftShooter = new WPI_TalonFX(Constants.SHOOTER_MOTOR_TWO);
    WPI_VictorSPX lowerTowerMotor = new WPI_VictorSPX(Constants.LOWER_TOWER_MOTOR);
    WPI_VictorSPX upperTowerMotor = new WPI_VictorSPX(Constants.UPPER_TOWER_MOTOR);
    WPI_TalonSRX strayTowerMotor = new WPI_TalonSRX(Constants.STRAY_TOWER_MOTOR);

    MotorControllerGroup tower = new MotorControllerGroup(lowerTowerMotor, upperTowerMotor);
    MotorControllerGroup shooter = new MotorControllerGroup(rightShooter, leftShooter);

    public void shoot(double speed) {
        tower.set(0.7);
        shooter.set(speed);
    }
}
