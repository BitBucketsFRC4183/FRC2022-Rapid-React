package frc.robot.subsystem;

import frc.robot.config.Config;

public class AutonomousSubsystem extends BitBucketsSubsystem {

  private boolean isGyroReset;

  public AutonomousSubsystem(Config config) {
    super(config);
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
