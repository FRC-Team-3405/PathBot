package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.AnalogInput;

class BallProcessing {
    private DigitalInput breakLight = new DigitalInput(0);
    private WPI_VictorSPX towerFeed = new WPI_VictorSPX(0);

    public enum Feed {
        IN,
        OUT,
        OFF
    }


    public BallProcessing() {}

    public void setTowerFeed(Feed f) {
        if (f == Feed.IN) {
            towerFeed.set(ControlMode.PercentOutput, .5);
        } else if (f == Feed.OUT) {
            towerFeed.set(ControlMode.PercentOutput, -.5);
        } else if (f == Feed.OFF) {
            towerFeed.set(ControlMode.PercentOutput, 0);
        }
    }

    public boolean isBroken() {
        return breakLight.get();
    }
}