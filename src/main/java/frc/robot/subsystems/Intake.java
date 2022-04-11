package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
// import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Intake extends SubsystemBase {
    public static DoubleSolenoid extender;

    private static WPI_VictorSPX motor = new WPI_VictorSPX(Constants.INTAKE_VICTOR);
    private static WPI_TalonSRX orangeRight = new WPI_TalonSRX(Constants.ORANGE_RIGHT);
    private static WPI_TalonSRX orangeLeft = new WPI_TalonSRX(Constants.ORANGE_LEFT);

    public Intake(DoubleSolenoid retracter) {
        extender = retracter;
    }

    public static void setMotor(boolean on) {
        if (on) {
            motor.set(ControlMode.PercentOutput, .5); // Intake Motor
            orangeRight.set(ControlMode.PercentOutput, .3); // Spinny Orange Wheel
            orangeRight.set(ControlMode.PercentOutput, -.3); // Spinny Orange Wheel 2
        } else {
            motor.set(ControlMode.PercentOutput, 0);
            orangeRight.set(ControlMode.PercentOutput, 0);
            orangeLeft.set(ControlMode.PercentOutput, 0);
        }
    }

    public void extend() {
        extender.set(Value.kReverse);
    }

    public void retract() {
        extender.set(Value.kForward);
    }
    
}
