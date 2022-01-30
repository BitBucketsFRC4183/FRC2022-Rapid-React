package frc.robot.subsystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import frc.robot.config.Config;
import frc.robot.log.LogLevel;
import frc.robot.utils.MotorUtils;

public class ShooterSubsystem extends BitBucketsSubsystem {

  private CANSparkMax roller1;
  private CANSparkMax roller2;

  private float hubShootSpeedTop = 5400;
  private float hubShootSpeedBottom = 5200;

  //for shuffleboard config (this is ib)
  double topShooterSpeed;
  double bottonShooterSpeed;

  public ShooterSubsystem(Config config) {
    super(config);
  }


  public void shootTop() {
    logger().logString(LogLevel.GENERAL, "shoot_state", "TopShooting");
    roller1.getPIDController().setReference(hubShootSpeedTop, ControlType.kVelocity, MotorUtils.velocitySlot);
    roller2.getPIDController().setReference(hubShootSpeedBottom, ControlType.kVelocity, MotorUtils.velocitySlot);
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

    logger().subscribeNum("bottonShooterSpeed",(e) -> {
        bottonShooterSpeed = e.doubleValue();
    }, 5200.0);

    roller1 = MotorUtils.makeSpark(config.shooter.roller1);
    roller2 = MotorUtils.makeSpark(config.shooter.roller2);
  }

  @Override
  public void periodic() {}

  @Override
  public void disable() {
    roller1.set(0);
    roller2.set(0);
  }
}
