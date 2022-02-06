package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.REVPhysicsSim;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Robot;
import frc.robot.config.Config;
import frc.robot.log.*;
import frc.robot.utils.MotorUtils;
import frc.swervelib.SimConstants;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.math.util.Units;

public class ShooterSubsystem extends BitBucketsSubsystem {

  private CANSparkMax roller1;
  private CANSparkMax roller2;
  private TalonSRX feeder1;
  private TalonSRX feeder2;

  private final Changeable<Double> topSpeed = BucketLog.changeable(Put.DOUBLE, "shooter/topShooterSpeed", 5400.0);
  private final Changeable<Double> bottomSpeed = BucketLog.changeable(Put.DOUBLE, "shooter/bottomShooterSpeed", -5200.0);
  private final Changeable<Double> feeder1PO = BucketLog.changeable(Put.DOUBLE, "shooter/feederOnePercentOutput", -0.5);
  private final Changeable<Double> feeder2PO = BucketLog.changeable(Put.DOUBLE, "shooter/feederTwoPercentOutput", 0.5);
  private float hubShootSpeedDeadband = 100;

  private final Loggable<String> shootState = BucketLog.loggable(Put.STRING, "shooter/shootState");
  private final Loggable<Double> roller1OutputVelLoggable = BucketLog.loggable(Put.DOUBLE, "shooter/Roller1OutputVel");
  private final Loggable<Double> roller2OutputVelLoggable = BucketLog.loggable(Put.DOUBLE, "shooter/Roller2OutputVel");

  FlywheelSim flywheelSim;
  EncoderSim encoderSim;

  enum ShooterState {
    STOPPED,
    TOP,
    LOW,
    TARMAC,
  }

  ShooterState shooterState = ShooterState.STOPPED;

  public ShooterSubsystem(Config config) {
    super(config);
  }

  public void stopShoot() {
    shootState.log("Idling");
    shooterState = ShooterState.STOPPED;

    roller1.set(0);
    roller2.set(0);
    feeder1.set(ControlMode.PercentOutput, 0);
    feeder2.set(ControlMode.PercentOutput, 0);
  }

  public void shootTop() {
    shootState.log("TopShooting");

    roller1.getPIDController().setReference(topSpeed.currentValue(), ControlType.kVelocity, MotorUtils.velocitySlot);
    roller2.getPIDController().setReference(bottomSpeed.currentValue(), ControlType.kVelocity, MotorUtils.velocitySlot);
    shooterState = ShooterState.TOP;
  }

  public void shootLow() {
    shootState.log("LowShooting");
    shooterState = ShooterState.LOW;
  }

  public void shootTarmac() {
    shootState.log("TarmacShooting");
    shooterState = ShooterState.TARMAC;
  }

  void turnOnFeeders() {
    feeder1.set(ControlMode.PercentOutput, feeder1PO.currentValue());
    feeder2.set(ControlMode.PercentOutput, feeder2PO.currentValue());
  }

  void turnOffFeeders() {
    feeder1.set(ControlMode.PercentOutput, 0);
    feeder2.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void init() {
    roller1 = MotorUtils.makeSpark(config.shooter.roller1);
    roller2 = MotorUtils.makeSpark(config.shooter.roller2);
    feeder1 = new WPI_TalonSRX(config.shooterFeeder1_ID);
    feeder2 = new WPI_TalonSRX(config.shooterFeeder2_ID);

    if (Robot.isSimulation())
    {
      // REVPhysicsSim.getInstance().addSparkMax(roller1, DCMotor.getNEO(1));
      flywheelSim = new FlywheelSim(DCMotor.getNEO(1), 3, 0.008);

      Encoder encoder = new Encoder(2, 3);
      encoder.reset();
      encoderSim = new EncoderSim(encoder);
    }
  }

  boolean motorIsInSpeedDeadband(CANSparkMax motor, double speed) {
    return (
      (motor.getEncoder().getVelocity() <= speed + hubShootSpeedDeadband) &&
      (motor.getEncoder().getVelocity() >= speed - hubShootSpeedDeadband)
    );
  }

  public boolean isUpToSpeed() {
    return true || motorIsInSpeedDeadband(roller1, topSpeed.currentValue()) && motorIsInSpeedDeadband(roller2, bottomSpeed.currentValue());
  }

  @Override
  public void periodic() {
    if (shooterState != ShooterState.STOPPED && isUpToSpeed()) {
      turnOnFeeders();
    } else {
      turnOffFeeders();
    }
  }

  @Override
  public void simulationPeriodic()
  {
    REVPhysicsSim.getInstance().run();

    flywheelSim.setInput(roller1.get() * this.config.maxVoltage);
    flywheelSim.update(SimConstants.SIM_SAMPLE_RATE_SEC);
    encoderSim.setRate(flywheelSim.getAngularVelocityRadPerSec());

    // encoderSim.get

    // roller1.getEncoder().setPosition(Units.radiansPerSecondToRotationsPerMinute(encoderSim.getRate()));

    // flywheelSim.setInput(motor.get() * RobotController.getBatteryVoltage());
    // flywheelSim.update(Constants.kRobotMainLoopPeriod);
    // encoderSim.setRate(flywheelSim.getAngularVelocityRadPerSec());

    roller1OutputVelLoggable.log(roller1.getEncoder().getVelocity());
    roller2OutputVelLoggable.log(roller2.getEncoder().getVelocity());
  }

  @Override
  public void disable() {
    roller1.set(0);
    roller2.set(0);
    feeder1.set(ControlMode.PercentOutput, 0);
    feeder2.set(ControlMode.PercentOutput, 0);
  }
}
