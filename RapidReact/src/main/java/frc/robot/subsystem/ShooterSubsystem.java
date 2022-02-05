package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import frc.robot.config.Config;
import frc.robot.log.*;
import frc.robot.utils.MotorUtils;

public class ShooterSubsystem extends BitBucketsSubsystem {

  private CANSparkMax roller1;
  private CANSparkMax roller2;
  private TalonSRX feeder1;
  private TalonSRX feeder2;

  private final Changeable<Double> topSpeed = BucketLog.changeable(Put.DOUBLE, "shooter/topShooterSpeed", 5400.0);
  private final Changeable<Double> bottomSpeed = BucketLog.changeable(Put.DOUBLE, "shooter/bottomShooterSpeed", 5200.0);
  private final Changeable<Double> feeder1PO = BucketLog.changeable(Put.DOUBLE, "shooter/feederOnePercentOutput", 0.5);
  private final Changeable<Double> feeder2PO = BucketLog.changeable(Put.DOUBLE, "shooter/feederTwoPercentOutput", 0.5);

  private final Loggable<String> shootState = BucketLog.loggable(Put.STRING, "shooter/shootState");

  public ShooterSubsystem(Config config) {
    super(config);
  }

  public void shootTop() {
    shootState.log("TopShooting");
    roller1.getPIDController().setReference(topSpeed.currentValue(), ControlType.kVelocity, MotorUtils.velocitySlot);
    roller2.getPIDController().setReference(bottomSpeed.currentValue(), ControlType.kVelocity, MotorUtils.velocitySlot);

    //?????
    //In place of isUpToSpeed()
    feeder1.set(ControlMode.PercentOutput, feeder1PO.currentValue());
    feeder2.set(ControlMode.PercentOutput, feeder2PO.currentValue());
  }

  public void stopShoot() {
    shootState.log("Idling");
    roller1.set(0);
    roller2.set(0);
    feeder1.set(ControlMode.Current, 0);
    feeder2.set(ControlMode.Current, 0);
  }

  public void shootLow() {
    shootState.log("LowShooting");
  }

  public void shootTarmac() {
    shootState.log("TarmacShooting");
  }

  @Override
  public void init() {
    roller1 = MotorUtils.makeSpark(config.shooter.roller1);
    roller2 = MotorUtils.makeSpark(config.shooter.roller2);
    feeder1 = new WPI_TalonSRX(config.shooterFeeder1_ID);
    feeder2 = new WPI_TalonSRX(config.shooterFeeder2_ID);
  }

  public boolean isUpToSpeed() {
    return (roller1.getEncoder().getVelocity() > topSpeed.currentValue()) && (roller2.getEncoder().getVelocity() > bottomSpeed.currentValue());
  }

  @Override
  public void periodic() {}

  @Override
  public void disable() {
    roller1.set(0);
    roller2.set(0);
  }
}
