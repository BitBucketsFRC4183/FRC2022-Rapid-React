package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.config.Config;

public class ShooterSubsystem extends BitBucketsSubsystem {

  private TalonSRX doSomethingElseIdk;
  private TalonSRX launchTheBalls;

  public ShooterSubsystem(Config config) {
    super(config);
  }

  public void shootTop() {



  }
  public void stopShootTop() {

  }

  public void shootTarmac() {

  }

  public void stopShootTarmac() {

  }

  @Override
  public void init() {}

  @Override
  public void periodic() {}

  @Override
  public void disable() {}
}
