package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
public class Shooter extends SubsystemBase {
    WPI_TalonFX rightShooter = new WPI_TalonFX(Constants.SHOOTER_MOTOR);
    WPI_TalonFX leftShooter = new WPI_TalonFX(Constants.SHOOTER_MOTOR_TWO);
    WPI_TalonSRX lowerTowerMotor = new WPI_TalonSRX(Constants.LOWER_TOWER_MOTOR);
    WPI_TalonSRX upperTowerMotor = new WPI_TalonSRX(Constants.UPPER_TOWER_MOTOR);
    WPI_VictorSPX strayTowerMotor = new WPI_VictorSPX(Constants.STRAY_TOWER_MOTOR);

    MotorControllerGroup tower = new MotorControllerGroup(lowerTowerMotor, upperTowerMotor);
    MotorControllerGroup shooter = new MotorControllerGroup(rightShooter, leftShooter);

    public Shooter() {
        leftShooter.configStatorCurrentLimit(
            new StatorCurrentLimitConfiguration(true, 70, 90, 1.0)
        );
        leftShooter.configSupplyCurrentLimit(
            new SupplyCurrentLimitConfiguration(true, 39, 40, 10)
        );
        rightShooter.configStatorCurrentLimit(
            new StatorCurrentLimitConfiguration(true, 70, 90, 1.0)
        );
        rightShooter.configSupplyCurrentLimit(
            new SupplyCurrentLimitConfiguration(true, 39, 40, 10)
        );
    }

    public void shoot(double speed) {
        rightShooter.set(speed);
        leftShooter.set(speed);
    }

    public void tower(double speed) {
        strayTowerMotor.set(speed);
        lowerTowerMotor.set(speed);
        upperTowerMotor.set(speed);
    }
}
