package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import frc.robot.config.Config;
import frc.robot.log.LogLevel;
import frc.robot.utils.MotorUtils;

public class ShooterSubsystem extends BitBucketsSubsystem {

  private CANSparkMax roller1;
  private CANSparkMax roller2;
  private TalonSRX feeder1;
  private TalonSRX feeder2;

  private float hubShootSpeedTop = 5400;
  private float hubShootSpeedBottom = 5200;

  //for shuffleboard config (this is ib)
  double topShooterSpeed;
  double bottomShooterSpeed;
  double feeder1PercentOutput = .5;
  double feeder2PercentOutput = .5;

  public ShooterSubsystem(Config config) {
    super(config);
  }

  public void shootTop() {
    logger().logString(LogLevel.GENERAL, "shoot_state", "TopShooting");
    roller1.getPIDController().setReference(hubShootSpeedTop, ControlType.kVelocity, MotorUtils.velocitySlot);
    roller2.getPIDController().setReference(hubShootSpeedBottom, ControlType.kVelocity, MotorUtils.velocitySlot);
    if (true) { //In place of isUpToSpeed()
      feeder1.set(ControlMode.PercentOutput, feeder1PercentOutput);
      feeder2.set(ControlMode.PercentOutput, feeder2PercentOutput);
    }
  }

  public void stopShoot() {
    logger().logString(LogLevel.GENERAL, "shoot_state", "Idling");
    roller1.set(0);
    roller2.set(0);
    feeder1.set(ControlMode.Current, 0);
    feeder2.set(ControlMode.Current, 0);
  }

  public void shootLow() {
    logger().logString(LogLevel.GENERAL, "shoot_state", "LowShooting");
  }

  public void shootTarmac() {
    logger().logString(LogLevel.GENERAL, "shoot_state", "TarmacShooting");
  }

  @Override
  public void init() {
    logger()
            .subscribeNum(
                    "HubShootSpeedTop",
                    value -> {
                      this.hubShootSpeedTop = value.floatValue();
                    },
                    5400.0

            );

    logger()
            .subscribeNum(
                    "HubShootSpeedBottom",
                    value -> {
                      this.hubShootSpeedTop = value.floatValue();
                    },
                    5200.0

            );
    
    logger().subscribeNum("topShooterSpeed",(e) -> {
        topShooterSpeed = e.doubleValue();
      }, 5400.0);

    logger().subscribeNum("bottomShooterSpeed",(e) -> {
        bottomShooterSpeed = e.doubleValue();
    }, 5200.0);

    logger()
            .subscribeNum(
                    "feeder1PercentOutput",
                    value -> {
                      this.feeder1PercentOutput = value.floatValue();
                    },
                    feeder1PercentOutput

            );
    
    logger()
    .subscribeNum(
            "feeder2PercentOutput",
            value -> {
              this.feeder2PercentOutput = value.floatValue();
            },
            feeder2PercentOutput

    );

    roller1 = MotorUtils.makeSpark(config.shooter.roller1);
    roller2 = MotorUtils.makeSpark(config.shooter.roller2);
    feeder1 = new WPI_TalonSRX(config.shooterFeeder1_ID);
    feeder2 = new WPI_TalonSRX(config.shooterFeeder2_ID);
  }

  public boolean isUpToSpeed() {
    return (roller1.getEncoder().getVelocity() > topShooterSpeed) && (roller2.getEncoder().getVelocity() > bottomShooterSpeed);
  }

  @Override
  public void periodic() {}

  @Override
  public void disable() {
    roller1.set(0);
    roller2.set(0);
  }
}
