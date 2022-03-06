package frc.robot.subsystem;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.config.Config;

public class AutonomousSubsystem extends BitBucketsSubsystem {

  public Field2d field;
  private boolean isGyroReset;

  public AutonomousSubsystem(Config config) {
    super(config);
  }

  public void setTrajectory(Trajectory trajectory) {
    field.getObject("traj").setTrajectory(trajectory);
  }

  @Override
  public void init()
  {
    this.isGyroReset = false;
  }

  public void setGyroReset()
  {
    this.isGyroReset = true;
  }

  public boolean isGyroReset()
  {
    return this.isGyroReset;
  }

  @Override
  public void periodic() {}

  @Override
  public void disable() {}
}
