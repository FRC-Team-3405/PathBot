package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

// import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
// import edu.wpi.first.wpilibj.AnalogInput;

public class BallProcessing {
   // private static DigitalInput breakLight = new DigitalInput(Constants.BREAKLIGHT_PORT);
    private WPI_VictorSPX towerFeed = new WPI_VictorSPX(Constants.TOWER_FEED_MOTOR);

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

    //public static boolean isBroken() {
        //return breakLight.get();
    }
//}