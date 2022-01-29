package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.config.Config;
import frc.robot.log.LogLevel;
import frc.robot.utils.MotorUtils;

public class ShooterSubsystem extends BitBucketsSubsystem {

  private CANSparkMax roller1;
  private CANSparkMax roller2;

  private float hubShootSpeedTop = 5400;
  private float hubShootSpeedBottom = 5200;

  public ShooterSubsystem(Config config) {
    super(config);
  }

  public void shootTop() {
    logger().logString(LogLevel.GENERAL, "shoot_state", "TopShooting");
    this.hubShootSpeedTop = (float) SmartDashboard.getNumber("ShooterSubsystem/HubShootSpeedTop", 0);
    this.hubShootSpeedBottom = (float) SmartDashboard.getNumber("ShooterSubsystem/HubShootSpeedBottom", 0);
    roller1.getPIDController().setReference(hubShootSpeedTop, ControlType.kVelocity, MotorUtils.velocitySlot);
    roller2.getPIDController().setReference(hubShootSpeedBottom, ControlType.kVelocity, MotorUtils.velocitySlot);
  }

  public void stopShoot() {
    logger().logString(LogLevel.GENERAL, "shoot_state", "Idling");
    this.hubShootSpeedTop = (float) SmartDashboard.getNumber("ShooterSubsystem/HubShootSpeedTop", 0);
    this.hubShootSpeedBottom = (float) SmartDashboard.getNumber("ShooterSubsystem/HubShootSpeedBottom", 0);
    roller1.set(0);
    roller2.set(0);
  }

  public void shootLow() {
    this.hubShootSpeedTop = (float) SmartDashboard.getNumber("ShooterSubsystem/HubShootSpeedTop", 0);
    this.hubShootSpeedBottom = (float) SmartDashboard.getNumber("ShooterSubsystem/HubShootSpeedBottom", 0);
    logger().logString(LogLevel.GENERAL, "shoot_state", "LowShooting");
  }

  public void shootTarmac() {
    this.hubShootSpeedTop = (float) SmartDashboard.getNumber("ShooterSubsystem/HubShootSpeedTop", 0);
    this.hubShootSpeedBottom = (float) SmartDashboard.getNumber("ShooterSubsystem/HubShootSpeedBottom", 0);
    logger().logString(LogLevel.GENERAL, "shoot_state", "TarmacShooting");
  }

  @Override
  public void init() {
    logger().logNum(LogLevel.GENERAL, "HubShootSpeedTop", hubShootSpeedTop);
    logger().logNum(LogLevel.GENERAL, "HubShootSpeedBottom", hubShootSpeedBottom);
    this.hubShootSpeedTop = (float) SmartDashboard.getNumber("ShooterSubsystem/HubShootSpeedTop", 0);
    this.hubShootSpeedBottom = (float) SmartDashboard.getNumber("ShooterSubsystem/HubShootSpeedBottom", 0);
    // logger()
    //   .subscribeNum(
    //     "HubShootSpeedTop",
    //     value -> {
    //       this.hubShootSpeedTop = value.floatValue();
    //     }
    //   );

    // logger()
    //   .subscribeNum(
    //     "HubShootSpeedBottom",
    //     value -> {
    //       this.hubShootSpeedBottom = value.floatValue();
    //     }
    //   );
    roller1 = MotorUtils.makeSpark(config.shooter.roller1);
    roller2 = MotorUtils.makeSpark(config.shooter.roller2);
  }

  @Override
  public void periodic() {}

  @Override
  public void disable() {}
}
