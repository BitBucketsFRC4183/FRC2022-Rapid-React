package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import frc.robot.config.Config;
import frc.robot.log.LogLevel;
import frc.robot.utils.MotorUtils;

public class ShooterSubsystem extends BitBucketsSubsystem {

  private CANSparkMax roller1;
  private CANSparkMax roller2;

  public ShooterSubsystem(Config config) {
    super(config);
  }

  public void shootTop() {
    logger().logString(LogLevel.GENERAL, "shoot_state", "TopShooting");
    roller1.getPIDController().setReference(5000, ControlType.kVelocity, 0);
    roller2.getPIDController().setReference(5000, ControlType.kVelocity, 0);
  }

  public void stopShoot() {
    logger().logString(LogLevel.GENERAL, "shoot_state", "Idling");
    roller1.set(0);
    roller2.set(0);
  }

  public void shootLow() {
    logger().logString(LogLevel.GENERAL, "shoot_state", "LowShooting");
  }

  public void shootTarmac() {
    logger().logString(LogLevel.GENERAL, "shoot_state", "TarmacShooting");
  }

  @Override
  public void init() {
    roller1 = MotorUtils.makeSpark(config.shooter.roller1);
    roller2 = MotorUtils.makeSpark(config.shooter.roller2);
  }

  @Override
  public void periodic() {}

  @Override
  public void disable() {}
}
