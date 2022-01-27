package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.config.Config;
import frc.robot.log.LogLevel;

public class ShooterSubsystem extends BitBucketsSubsystem {

  private TalonSRX roller1;
  private TalonSRX roller2;

  public ShooterSubsystem(Config config) {
    super(config);
  }

  public void shootTop() {
    logger().logString(LogLevel.GENERAL, "shoot_state", "TopShooting");
  }

  public void stopShootTop() {
    logger().logString(LogLevel.GENERAL, "shoot_state", "Idling");
  }

  public void shootLow() {
    logger().logString(LogLevel.GENERAL, "shoot_state", "LowShooting");
  }

  public void stopShootLow() {
    logger().logString(LogLevel.GENERAL, "shoot_state", "Idling");
  }

  public void shootTarmac() {
    logger().logString(LogLevel.GENERAL, "shoot_state", "TarmacShooting");
  }

  public void stopShootTarmac() {
    logger().logString(LogLevel.GENERAL, "shoot_state", "Idling");
  }

  @Override
  public void init() {}

  @Override
  public void periodic() {}

  @Override
  public void disable() {}
}
