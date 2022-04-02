package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Intake extends SubsystemBase {
    private DoubleSolenoid extender = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 
        Constants.EXTENDER_FORWARD, Constants.EXTENDER_BACKWARDS);

    private WPI_VictorSPX motor = new WPI_VictorSPX(Constants.INTAKE_VICTOR);

    public Intake() {}

    public void setMotor(boolean on) {
        if (on) {
            motor.set(ControlMode.PercentOutput, .5);
        } else {
            motor.set(ControlMode.PercentOutput, 0);
        }
    }

    public void extend() {
        extender.set(Value.kForward);
    }

    public void retract() {
        extender.set(Value.kReverse);
    }
    
}
