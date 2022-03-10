package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.REVPhysicsSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Robot;
import frc.robot.config.Config;
import frc.robot.log.*;
import frc.robot.utils.MotorUtils;

public class ShooterSubsystem extends BitBucketsSubsystem {

  private CANSparkMax shooterTop;
  private CANSparkMax shooterBottom;
  private TalonSRX feeder;

  private final Changeable<Double> topSpeed = BucketLog.changeable(Put.DOUBLE, "shooter/topShooterSpeed", 5000.0);
  private final Changeable<Double> bottomSpeed = BucketLog.changeable(
    Put.DOUBLE,
    "shooter/bottomShooterSpeed",
    -3000.0
  );
  private final Changeable<Double> topSpeedLow = BucketLog.changeable(Put.DOUBLE, "shooter/topShooterSpeedLow", 2300.0);
  private final Changeable<Double> bottomSpeedLow = BucketLog.changeable(
    Put.DOUBLE,
    "shooter/bottomShooterSpeedLow",
    -3000.0
  );
  private final Changeable<Double> feederPO = BucketLog.changeable(Put.DOUBLE, "shooter/feederPercentOutput", -0.5);
  private final Changeable<Double> feederHoldPO = BucketLog.changeable(Put.DOUBLE, "shooter/feederHoldPercentOutput", -0.7);
  private float hubSpinUpSpeedDeadband = 300;

  private final Loggable<String> shootState = BucketLog.loggable(Put.STRING, "shooter/shootState");
  private final Loggable<Double> roller1OutputVelLoggable = BucketLog.loggable(Put.DOUBLE, "shooter/Roller1OutputVel");
  private final Loggable<Double> roller2OutputVelLoggable = BucketLog.loggable(Put.DOUBLE, "shooter/Roller2OutputVel");

  private final Loggable<Double> topShooterSpeed = BucketLog.loggable(Put.DOUBLE, "shooter/topShooterActualSpeed");
  private final Loggable<Double> bottomShooterSpeed = BucketLog.loggable(Put.DOUBLE, "shooter/bottomShooterActualSpeed");

  private final Loggable<Double> topShooterError = BucketLog.loggable(Put.DOUBLE, "shooter/topShooterError");
  private final Loggable<Double> bottomShooterError = BucketLog.loggable(Put.DOUBLE, "shooter/bottomShooterError");

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

  public boolean isShooting(){
    return shooterState != ShooterState.STOPPED;
  }

  public void stopShoot() {
    shootState.log("Idling");
    shooterState = ShooterState.STOPPED;

    shooterTop.set(0);
    shooterBottom.set(0);
    turnOffFeeders();
  }

  public void spinUpTop() {
    shootState.log("TopShooting");

    shooterTop.getPIDController().setReference(topSpeed.currentValue(), ControlType.kVelocity, MotorUtils.velocitySlot);
    shooterBottom.getPIDController().setReference(bottomSpeed.currentValue(), ControlType.kVelocity, MotorUtils.velocitySlot);
    shooterState = ShooterState.TOP;
  }

  public void shootLow() {
    shootState.log("LowShooting");
    shooterTop.getPIDController().setReference(topSpeedLow.currentValue(), ControlType.kVelocity, MotorUtils.velocitySlot);
    shooterBottom
      .getPIDController()
      .setReference(bottomSpeedLow.currentValue(), ControlType.kVelocity, MotorUtils.velocitySlot);
    shooterState = ShooterState.LOW;
  }

  public void shootTarmac() {
    shootState.log("TarmacShooting");
    shooterState = ShooterState.TARMAC;
  }

  public void turnOnFeeders() {
    feeder.set(ControlMode.PercentOutput, feederPO.currentValue());
  }

  public void turnOffFeeders() {
    feeder.set(ControlMode.PercentOutput, 0);
  }

  public void antiFeed() {
    feeder.set(ControlMode.PercentOutput, -feederHoldPO.currentValue());
  }
  
  @Override
  public void init() {
    shooterTop = MotorUtils.makeSpark(config.shooter.shooterTop);
    shooterBottom = MotorUtils.makeSpark(config.shooter.shooterBottom);
    feeder = new WPI_TalonSRX(config.shooterFeeder_ID);

    //limit the voltage of the feeder motors
    feeder.configVoltageCompSaturation(11);
    feeder.enableVoltageCompensation(true);

    if (Robot.isSimulation()) {
      // REVPhysicsSim.getInstance().addSparkMax(roller1, DCMotor.getNEO(1));
      flywheelSim = new FlywheelSim(DCMotor.getNEO(1), 3, 0.008);

      Encoder encoder = new Encoder(2, 3);
      encoder.reset();
      encoderSim = new EncoderSim(encoder);
    }
  }

  boolean motorIsInSpeedDeadband(CANSparkMax motor, double speed) {
    return (
      (motor.getEncoder().getVelocity() <= speed + hubSpinUpSpeedDeadband) &&
      (motor.getEncoder().getVelocity() >= speed - hubSpinUpSpeedDeadband)
    );
  }

  public boolean isUpToSpeed() {
    return (
      // true ||
      motorIsInSpeedDeadband(shooterTop, topSpeed.currentValue()) &&
      motorIsInSpeedDeadband(shooterBottom, bottomSpeed.currentValue())
    );
  }

  @Override
  public void periodic() {
    topShooterSpeed.log(LogLevel.GENERAL, shooterTop.getEncoder().getVelocity());
    bottomShooterSpeed.log(LogLevel.GENERAL, shooterBottom.getEncoder().getVelocity());

    double topError;
    double bottomError;
    if (isShooting())
    {
      topError = shooterTop.getEncoder().getVelocity() - topSpeed.currentValue();
      bottomError = shooterBottom.getEncoder().getVelocity() - bottomSpeed.currentValue();
    }
    else
    {
      topError = shooterTop.getEncoder().getVelocity();
      bottomError = shooterTop.getEncoder().getVelocity();
    }

    topShooterError.log(LogLevel.GENERAL, topError);
    bottomShooterError.log(LogLevel.GENERAL, bottomError);
  }

  @Override
  public void simulationPeriodic() {
    REVPhysicsSim.getInstance().run();

    flywheelSim.setInput(shooterTop.get() * this.config.maxVoltage);
    flywheelSim.update(0.02); //Used to be SimConstants.SIM_SAMPLE_RATE_SEC
    encoderSim.setRate(flywheelSim.getAngularVelocityRadPerSec());

    // encoderSim.get

    // roller1.getEncoder().setPosition(Units.radiansPerSecondToRotationsPerMinute(encoderSim.getRate()));

    // flywheelSim.setInput(motor.get() * RobotController.getBatteryVoltage());
    // flywheelSim.update(Constants.kRobotMainLoopPeriod);
    // encoderSim.setRate(flywheelSim.getAngularVelocityRadPerSec());

    roller1OutputVelLoggable.log(shooterTop.getEncoder().getVelocity());
    roller2OutputVelLoggable.log(shooterBottom.getEncoder().getVelocity());
  }

  @Override
  public void disable() {
    shooterTop.set(0);
    shooterBottom.set(0);
    feeder.set(ControlMode.PercentOutput, 0);
  }
}
