package frc.robot.commands.autocommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class SpinShooterMotor extends CommandBase {
    Timer t;
    public SpinShooterMotor() {
        addRequirements(RobotContainer.m_robotDrive);
    }

    @Override
    public void initialize() {
        t = new Timer();
        t.start();
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      RobotContainer.m_shooter.shoot(-0.85);
      if (t.get() > 1.4) {
        RobotContainer.m_shooter.tower(-0.5);
      }
      else {
        RobotContainer.m_shooter.tower(0);
      }
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        t.stop();
        RobotContainer.m_shooter.shoot(0);
        RobotContainer.m_shooter.tower(0);
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return t.hasElapsed(2.0);
    }
}
