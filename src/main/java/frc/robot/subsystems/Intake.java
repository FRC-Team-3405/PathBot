package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
// import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
// import edu.wpi.first.wpilibj.PneumaticsModuleType;
// import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

public class Intake extends SubsystemBase {
    public static DoubleSolenoid extender;
    public static Boolean extended = false;
    

    private static WPI_TalonSRX motor = new WPI_TalonSRX(Constants.INTAKE_VICTOR);
    //private static WPI_TalonSRX orangeRight = new WPI_TalonSRX(Constants.ORANGE_RIGHT);
    //private static WPI_TalonSRX orangeLeft = new WPI_TalonSRX(Constants.ORANGE_LEFT);

    public Intake() {
        extender = new DoubleSolenoid( PneumaticsModuleType.CTREPCM,Constants.INTAKE_IN,Constants.INTAKE_OUT);
        extender.set(kForward);
    }

    public void setMotor() {
        //** TALONSRX CURRENT LIMITS (Intake Motor) */
        motor.configPeakCurrentLimit(30); // don't activate current limit until current exceeds 30 A ...
        motor.configPeakCurrentDuration(100); // ... for at least 100 ms
        motor.configContinuousCurrentLimit(15); // once current-limiting is actived, hold at 20A
        motor.enableCurrentLimit(true);

        if (extended) {
            System.out.println("Intake Motor On");
            motor.set(ControlMode.PercentOutput, -1.0); // Intake Motor
            //orangeRight.set(ControlMode.PercentOutput, 0.3); // Spinny Orange Wheel
            //orangeRight.set(ControlMode.PercentOutput, -0.3); // Spinny Orange Wheel 2
        } else {
            motor.set(ControlMode.PercentOutput, 0.0);
            //orangeRight.set(ControlMode.PercentOutput, 0.0);
            //orangeLeft.set(ControlMode.PercentOutput, 0.0);
        }
    }

    public void extend() {
        //extender.set(Value.kReverse);
        
        extender.toggle();
        extended = !extended;
        System.out.println(extended);
        // if (extended){
        //     motor.set(ControlMode.PercentOutput,.5);
        // }
        // else{
        //     motor.set(ControlMode.PercentOutput,0);
        // }
        
    }

    // public void retract() {
    //     //extender.set(Value.kForward);
    //     extender.toggle();
    //     extenderState = "In";
    // }

    
}
