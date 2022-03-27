package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
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

  private final Changeable<Double> topSpeedHigh = BucketLog.changeable(Put.DOUBLE, "shooter/topShooterSpeed", 2000.0);
  private final Changeable<Double> bottomSpeedHigh = BucketLog.changeable(
    Put.DOUBLE,
    "shooter/bottomShooterSpeed",
    4250.0
  );
  private final Changeable<Double> topSpeedLow = BucketLog.changeable(Put.DOUBLE, "shooter/topShooterSpeedLow", 2000.0);
  private final Changeable<Double> bottomSpeedLow = BucketLog.changeable(
    Put.DOUBLE,
    "shooter/bottomShooterSpeedLow",
    1600.0
  );

  private final Changeable<Double> feederPO = BucketLog.changeable(Put.DOUBLE, "shooter/feederPercentOutput", 0.7);
  private final Changeable<Double> feederHoldPO = BucketLog.changeable(Put.DOUBLE, "shooter/feederHoldPercentOutput", 0.8);

  private float hubSpinUpSpeedDeadband = 20;
  private int upToSpeedCount = 0;

  private final Loggable<String> shootState = BucketLog.loggable(Put.STRING, "shooter/shootState");
  private final Loggable<Double> shooterTopOutputVelLoggable = BucketLog.loggable(Put.DOUBLE, "shooter/ShooterTopOutputVel");
  private final Loggable<Double> shooterBottomOutputVelLoggable = BucketLog.loggable(Put.DOUBLE, "shooter/ShooterBottomOutputVel");
  private final Loggable<Boolean> isUpToHighSpeed = BucketLog.loggable(Put.BOOL, "shooter/isUpToHighSpeed");

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
    isUpToHighSpeed.log(false);
    upToSpeedCount = 0;
    shooterTop.set(0);
    shooterBottom.set(0);
    turnOffFeeders();
  }

  public void spinUpTop() {
    shootState.log("TopShooting");

    shooterTop.getPIDController().setReference(topSpeedHigh.currentValue(), ControlType.kVelocity, MotorUtils.velocitySlot);
    shooterBottom.getPIDController().setReference(bottomSpeedHigh.currentValue(), ControlType.kVelocity, MotorUtils.velocitySlot);

    shooterState = ShooterState.TOP;
  }

  public void shootLow() {
    shootState.log("LowShooting");

    shooterTop.getPIDController().setReference(topSpeedLow.currentValue(), ControlType.kVelocity, MotorUtils.velocitySlot);
    shooterBottom.getPIDController().setReference(bottomSpeedLow.currentValue(), ControlType.kVelocity, MotorUtils.velocitySlot);

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
    feeder = MotorUtils.makeSRX(config.shooter.feeder);

    shooterTop.getPIDController().setOutputRange(0, 1, MotorUtils.velocitySlot);
    shooterBottom.getPIDController().setOutputRange(0, 1, MotorUtils.velocitySlot);

    shooterTop.setIdleMode(CANSparkMax.IdleMode.kCoast);
    shooterBottom.setIdleMode(CANSparkMax.IdleMode.kCoast);

    //limit the voltage of the feeder motors
    feeder.configVoltageCompSaturation(11);
    feeder.enableVoltageCompensation(true);

    shooterTop.enableVoltageCompensation(11);
    shooterBottom.enableVoltageCompensation(11);

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

  public boolean isUpToHighSpeed() {
    boolean state = (motorIsInSpeedDeadband(shooterTop, topSpeedHigh.currentValue()) && motorIsInSpeedDeadband(shooterBottom, bottomSpeedHigh.currentValue()));
    if (state) {
      upToSpeedCount++;
    } else {
      upToSpeedCount = 0;
    }
    isUpToHighSpeed.log(LogLevel.GENERAL, state);
    return (state && upToSpeedCount >= 3);
  }

  public boolean isUpToLowSpeed() {
    return (
            motorIsInSpeedDeadband(shooterTop, topSpeedLow.currentValue())  &&
            motorIsInSpeedDeadband(shooterBottom, topSpeedLow.currentValue())
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
      if (shooterState == ShooterState.LOW) {
        topError = shooterTop.getEncoder().getVelocity() - topSpeedLow.currentValue();
        bottomError = shooterBottom.getEncoder().getVelocity() - bottomSpeedLow.currentValue();
      } else {
        topError = shooterTop.getEncoder().getVelocity() - topSpeedHigh.currentValue();
        bottomError = shooterBottom.getEncoder().getVelocity() - bottomSpeedHigh.currentValue();
      }

    }
    else
    {
      topError = shooterTop.getEncoder().getVelocity();
      bottomError = shooterTop.getEncoder().getVelocity();
    }

//    SmartDashboard.putNumber("shooter/topShooterError", topError);
//    SmartDashboard.putNumber("shooter/bottomShooterError", bottomError);
//    SmartDashboard.putNumber("shooter/topPercentOutput", shooterTop.getAppliedOutput());
//    SmartDashboard.putNumber("shooter/bottomPercentOutput", shooterBottom.getAppliedOutput());
     topShooterError.log(LogLevel.DEBUG, topError);
     bottomShooterError.log(LogLevel.DEBUG, bottomError);
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

    shooterTopOutputVelLoggable.log(shooterTop.getEncoder().getVelocity());
    shooterBottomOutputVelLoggable.log(shooterBottom.getEncoder().getVelocity());
  }

  @Override
  public void disable() {
    shooterTop.set(0);
    shooterBottom.set(0);
    feeder.set(ControlMode.PercentOutput, 0);
  }
}
