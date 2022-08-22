package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Robot;
import frc.robot.config.Config;
import frc.robot.config.MotorConfig;
import frc.robot.log.*;
import frc.robot.simulator.CTREPhysicsSim;
import frc.robot.utils.MotorUtils;

public class ClimberSubsystem extends BitBucketsSubsystem {

  // state machine diagram
  // https://i.imgur.com/zE1pEXn.png
  enum ClimbState {
    Idle,
    ExtendPartial,
    ExtendFull,
    Retract,
  }

  ClimbState currentClimbState;

  WPI_TalonSRX climberLeft = new WPI_TalonSRX(config.climberMotor_IDLeft);
  WPI_TalonSRX climberRight = new WPI_TalonSRX(config.climberMotor_IDRight);
  MotorConfig leaderConfig = config.climber.climberLeft;
  MotorConfig followerConfig = config.climber.climberRight;

  private boolean autoClimb; // is autoclimb enabled
  private boolean autoClimbPressed = false; // is the autoclimb button currently being pressed

  private boolean autoClimbStopped = false; // if this is stopped, auto climb can't be turned on again

  private boolean climberExtending = false;
  DoubleSolenoid elevatorSolenoid;

  private int fullExtendPositionUprightRight = 20033;
  private int fullExtendPositionUprightLeft = 19850;
  private int partialExtendPosition = fullExtendPositionUprightRight / 2;
  private int fullRetractPosition = 0; // TODO: change this number

  // https://docs.ctre-phoenix.com/en/stable/ch16_ClosedLoop.html#mechanism-is-finished-command
  private int climbErrThreshold = 1000;
  private int climbLoopsToSettle = 10;
  private int withinThresholdLoops1 = 0;
  private int withinThresholdLoops2 = 0;

  private boolean climberTilted = false;

  private final Changeable<Double> climbOutput = BucketLog.changeable(Put.DOUBLE, "climber/climbOutput", 1.0 );

  private final Changeable<Double> climbRetractSlow = BucketLog.changeable(
    Put.DOUBLE,
    "climber/climbRetractSlow",
    -0.1
  );

  private final Loggable<String> climbState = BucketLog.loggable(Put.STRING, "climber/climbState");
  private final Loggable<Boolean> elevatorTiltedState = BucketLog.loggable(Put.BOOL, "climber/elevatorTiltedState");

  private final Loggable<Double> climberLeftError = BucketLog.loggable(Put.DOUBLE, "climber/climberLeftError");
  private final Loggable<Double> climberLeftPosition = BucketLog.loggable(Put.DOUBLE, "climber/climberLeftPosition");
  // private final Loggable<Double> climberLeftVoltage = BucketLog.loggable(Put.DOUBLE, "climber/climberLeftPosition");
  private final Loggable<Double> climberLeftVelocity = BucketLog.loggable(Put.DOUBLE, "climber/climberLeftVelocity");

  private final Loggable<Double> climberRightError = BucketLog.loggable(Put.DOUBLE, "climber/climberRightError");
  private final Loggable<Double> climberRightPosition = BucketLog.loggable(Put.DOUBLE, "climber/climberRightPosition");
  private final Loggable<Double> climberRightVelocity = BucketLog.loggable(Put.DOUBLE, "climber/climberRightVelocity");

  private final Loggable<Boolean> climberLeftRevLimitSwitchClosedLog = BucketLog.loggable(
    Put.BOOL,
    "climber/climbLeftRevLimitSwitchClosed"
  );
  private final Loggable<Boolean> climberRightRevLimitSwitchClosedLog = BucketLog.loggable(
    Put.BOOL,
    "climber/climbRightRevLimitSwitchClosed"
  );

  private final Loggable<Boolean> climberLeftFwdLimitSwitchClosedLog = BucketLog.loggable(
    Put.BOOL,
    "climber/climbLeftFwdLimitSwitchClosed"
  );
  private final Loggable<Boolean> climberRightFwdLimitSwitchClosedLog = BucketLog.loggable(
    Put.BOOL,
    "climber/climbRightFwdLimitSwitchClosed"
  );

  // private final Loggable<Double> climberRightVoltage = BucketLog.loggable(Put.DOUBLE, "climber/climberRightPosition");

  public ClimberSubsystem(Config config) {
    super(config);
  }

  @Override
  public void init() {
    System.out.println("Daniel Wu\nYas queens");
    climberLeft.set(ControlMode.MotionMagic, 0);
    climberRight.set(ControlMode.MotionMagic, 0);

    climberLeft.configFactoryDefault();
    climberRight.configFactoryDefault();

    climberLeft.setNeutralMode(NeutralMode.Brake);
    climberLeft.setNeutralMode(NeutralMode.Brake);

    // climberLeft = MotorUtils.makeSRX(leaderConfig);
    // climberRight = MotorUtils.makeSRX(config.climber.climberRight);
    // climberRight.follow(climberLeft, FollowerType.AuxOutput1);

    // Configure the left Talon's selected sensor as local QuadEncoder
    climberLeft.configSelectedFeedbackSensor(
      FeedbackDevice.QuadEncoder,
      MotorUtils.PRIMARY_PID_LOOP,
      MotorUtils.CONTROLLER_TIMEOUT_MS
    );
    // Configure the Remote Talon's selected sensor as a remote sensor for the right Talon
    // climberRight.configRemoteFeedbackFilter(
    //   climberLeft.getDeviceID(),
    //   RemoteSensorSource.TalonSRX_SelectedSensor,
    //   MotorUtils.REMOTE_0
    // );
    climberRight.configSelectedFeedbackSensor(
      FeedbackDevice.QuadEncoder,
      MotorUtils.PRIMARY_PID_LOOP,
      MotorUtils.CONTROLLER_TIMEOUT_MS
    );

    // Setup Sum signal to be used for Distance
    // climberRight.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor0);
    // climberRight.configSensorTerm(SensorTerm.Sum1, FeedbackDevice.CTRE_MagEncoder_Relative);

    // Setup Difference signal to be used for Turn
    // climberRight.configSensorTerm(SensorTerm.Diff0, FeedbackDevice.RemoteSensor0);
    // climberRight.configSensorTerm(SensorTerm.Diff1, FeedbackDevice.CTRE_MagEncoder_Relative);

    // Configure Sum [Sum of both QuadEncoders] to be used for Primary PID Index
    // climberRight.configSelectedFeedbackSensor(FeedbackDevice.SensorSum, MotorUtils.PRIMARY_PID_LOOP, 0);
    // Scale Feedback by 0.5 to half the sum of Distance
    // climberRight.configSelectedFeedbackCoefficient(
    //   0.5,
    //   MotorUtils.PRIMARY_PID_LOOP,
    //   MotorUtils.CONTROLLER_TIMEOUT_MS
    // );

    // Configure Difference [Difference between both QuadEncoders] to be used for Auxiliary PID Index
    // climberRight.configSelectedFeedbackSensor(
    //   FeedbackDevice.SensorDifference,
    //   MotorUtils.PRIMARY_PID_LOOP,
    //   MotorUtils.CONTROLLER_TIMEOUT_MS
    // );
    // Scale the Feedback Sensor using a coefficient
    // climberRight.configSelectedFeedbackCoefficient(1, MotorUtils.TURN_PID_LOOP, MotorUtils.CONTROLLER_TIMEOUT_MS);

    // Configure output and sensor direction
    climberLeft.setInverted(leaderConfig.inverted);
    climberLeft.setSensorPhase(leaderConfig.sensorPhase);
    climberRight.setInverted(followerConfig.inverted);
    climberRight.setSensorPhase(followerConfig.sensorPhase);

    // Set status frame periods to ensure we don't have stale data
    climberLeft.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20);
    climberLeft.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20);
    climberLeft.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20);
    climberLeft.setStatusFramePeriod(StatusFrame.Status_10_Targets, 20);
    climberRight.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 5);
    climberRight.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20);
    climberRight.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20);
    climberRight.setStatusFramePeriod(StatusFrame.Status_10_Targets, 20);

    // Configure neutral deadband
    climberLeft.configNeutralDeadband(MotorUtils.kNeutralDeadband);
    climberRight.configNeutralDeadband(MotorUtils.kNeutralDeadband);

    // Motion Magic Configurations
    climberLeft.configMotionAcceleration(leaderConfig.motionMagicAcceleration);
    climberLeft.configMotionCruiseVelocity(leaderConfig.motionMagicCruiseVelocity);
    climberRight.configMotionAcceleration(followerConfig.motionMagicAcceleration);
    climberRight.configMotionCruiseVelocity(followerConfig.motionMagicCruiseVelocity);

    /**
     * Max out the peak output (for all modes).
     * However you can limit the output of a given PID object with configClosedLoopPeakOutput().
     */
    climberLeft.configPeakOutputForward(+1.0);
    climberLeft.configPeakOutputReverse(-1.0);
    climberRight.configPeakOutputForward(+1.0);
    climberRight.configPeakOutputReverse(-1.0);

    /* FPID Gains for distance servo */
    climberLeft.config_kP(MotorUtils.positionSlot, leaderConfig.positionPIDF.getKP());
    climberLeft.config_kI(MotorUtils.positionSlot, leaderConfig.positionPIDF.getKI());
    climberLeft.config_kD(MotorUtils.positionSlot, leaderConfig.positionPIDF.getKD());
    climberLeft.config_kF(MotorUtils.positionSlot, leaderConfig.positionPIDF.getKF());
    climberLeft.config_IntegralZone(MotorUtils.positionSlot, leaderConfig.positionPIDF.getIZone());
    climberLeft.configClosedLoopPeakOutput(MotorUtils.positionSlot, leaderConfig.distancePeakOutput);
    climberLeft.configAllowableClosedloopError(MotorUtils.positionSlot, 0);
    climberLeft.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
    climberLeft.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
    climberLeft.overrideLimitSwitchesEnable(true);

    climberRight.config_kP(MotorUtils.positionSlot, followerConfig.positionPIDF.getKP());
    climberRight.config_kI(MotorUtils.positionSlot, followerConfig.positionPIDF.getKI());
    climberRight.config_kD(MotorUtils.positionSlot, followerConfig.positionPIDF.getKD());
    climberRight.config_kF(MotorUtils.positionSlot, followerConfig.positionPIDF.getKF());
    climberRight.config_IntegralZone(MotorUtils.positionSlot, followerConfig.positionPIDF.getIZone());
    climberRight.configClosedLoopPeakOutput(MotorUtils.positionSlot, followerConfig.distancePeakOutput);
    climberRight.configAllowableClosedloopError(MotorUtils.positionSlot, 0);
    climberRight.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
    climberRight.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
    climberRight.overrideLimitSwitchesEnable(true);

    /* FPID Gains for turn servo */
    // climberLeft.config_kP(MotorUtils.velocitySlot, leaderConfig.positionPIDF.getKP());
    // climberLeft.config_kI(MotorUtils.velocitySlot, leaderConfig.positionPIDF.getKI());
    // climberLeft.config_kD(MotorUtils.velocitySlot, leaderConfig.positionPIDF.getKD());
    // climberLeft.config_kF(MotorUtils.velocitySlot, leaderConfig.positionPIDF.getKF());
    // climberLeft.config_IntegralZone(MotorUtils.velocitySlot, leaderConfig.positionPIDF.getIZone());
    // climberRight.configClosedLoopPeakOutput(MotorUtils.velocitySlot, leaderConfig.turningPeakOutput);
    // climberRight.configAllowableClosedloopError(MotorUtils.velocitySlot, 0);

    /**
     * 1ms per loop.  PID loop can be slowed down if need be.
     * For example,
     * - if sensor updates are too slow
     * - sensor deltas are very small per update, so derivative error never gets large enough to be useful.
     * - sensor movement is very slow causing the derivative error to be near zero.
     */
    // TODO: Is there a reason this block is disabled?
    // int closedLoopTimeMs = 1;
    // climberRight.configClosedLoopPeriod(0, closedLoopTimeMs);
    // climberRight.configClosedLoopPeriod(1, closedLoopTimeMs);

    /**
     * configAuxPIDPolarity(boolean invert, int timeoutMs)
     * false means talon's local output is PID0 + PID1, and other side Talon is PID0 - PID1
     * true means talon's local output is PID0 - PID1, and other side Talon is PID0 + PID1
     */
    // climberRight.configAuxPIDPolarity(false);

    /* Initialize */
    // climberRight.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 10);

    if (Robot.isSimulation()) {
      CTREPhysicsSim.getInstance().addTalonSRX(climberLeft, .75, 5100, false);
      CTREPhysicsSim.getInstance().addTalonSRX(climberRight, .75, 5100, false);
    }
    if (config.enablePneumatics) {
      if (Robot.isSimulation()) {
        elevatorSolenoid =
          new DoubleSolenoid(PneumaticsModuleType.CTREPCM, config.elevatorSolenoid_ID1, config.elevatorSolenoid_ID2);
      } else {
        elevatorSolenoid =
          new DoubleSolenoid(PneumaticsModuleType.REVPH, config.elevatorSolenoid_ID1, config.elevatorSolenoid_ID2);
      }
      setElevatorTilted(false);
    }

    currentClimbState = ClimbState.Idle;
  }

  boolean isClimberAtSetpoint(double setpoint) {
    boolean climberLeftAtSetpoint = false;

    // if it's within the error threshold for a set amount of loops
    if (
      climberLeft.getClosedLoopError() < +climbErrThreshold && climberLeft.getClosedLoopError() > -climbErrThreshold
    ) {
      withinThresholdLoops1++;
      if (withinThresholdLoops1 > climbLoopsToSettle) {
        // and if its within the error threshold of the position
        if (
          climberLeft.getSelectedSensorPosition() > setpoint - climbErrThreshold &&
          climberLeft.getSelectedSensorPosition() < setpoint + climbErrThreshold
        ) {
          // then it's reached
          climberLeftAtSetpoint = true;
        }
      }
    } else {
      withinThresholdLoops1 = 0;
    }

    boolean climberRightAtSetpoint = false;

    if (
      climberRight.getClosedLoopError() < +climbErrThreshold && climberRight.getClosedLoopError() > -climbErrThreshold
    ) {
      withinThresholdLoops2++;
      if (withinThresholdLoops2 > climbLoopsToSettle) {
        if (
          climberRight.getSelectedSensorPosition() > setpoint - climbErrThreshold &&
          climberRight.getSelectedSensorPosition() < setpoint + climbErrThreshold
        ) {
          climberRightAtSetpoint = true;
        }
      }
    } else {
      withinThresholdLoops2 = 0;
    }

    return climberLeftAtSetpoint && climberRightAtSetpoint;
  }

  private void idleInit() {
    setElevatorTilted(false);
    autoRetractFull();
  }

  private void extendPartialInit() {
    setElevatorTilted(false);
    autoExtendPartial();
  }

  private void extendFullInit() {
    setElevatorTilted(true);
    autoExtendFull();
  }

  private void retractInit() {
    // TODO: ensure the hooks have engaged the bar before we begin retracting?
    // setElevatorTilted(false);
    autoRetractFull();
  }

  @Override
  public void periodic() {
    boolean climbLeftRevLimitSwitchClosed = climberLeft.getSensorCollection().isRevLimitSwitchClosed();
    boolean climbRightRevLimitSwitchClosed = climberRight.getSensorCollection().isRevLimitSwitchClosed();

    climbState.log(LogLevel.GENERAL, currentClimbState.toString());

    climberLeftPosition.log(LogLevel.GENERAL, climberLeft.getSelectedSensorPosition());
    climberLeftError.log(LogLevel.GENERAL, climberLeft.getClosedLoopError());
    climberLeftVelocity.log(LogLevel.GENERAL, climberLeft.getSelectedSensorVelocity());

    climberRightPosition.log(LogLevel.GENERAL, climberRight.getSelectedSensorPosition());
    climberRightError.log(LogLevel.GENERAL, climberRight.getClosedLoopError());
    climberRightVelocity.log(LogLevel.GENERAL, climberRight.getSelectedSensorVelocity());

    climberLeftRevLimitSwitchClosedLog.log(LogLevel.GENERAL, climbLeftRevLimitSwitchClosed);
    climberRightRevLimitSwitchClosedLog.log(LogLevel.GENERAL, climbRightRevLimitSwitchClosed);

    climberLeftFwdLimitSwitchClosedLog.log(
      LogLevel.GENERAL,
      climberLeft.getSensorCollection().isFwdLimitSwitchClosed()
    );
    climberRightFwdLimitSwitchClosedLog.log(
      LogLevel.GENERAL,
      climberRight.getSensorCollection().isFwdLimitSwitchClosed()
    );

    if (!climbLeftRevLimitSwitchClosed) {
      // if (!climbLeftEncoderZeroed) {
      //   climberLeft.set(ControlMode.PercentOutput, climbRetractSlow.currentValue());
      // }
    } else {
      climberLeft.setSelectedSensorPosition(0);
    }

    if (!climbRightRevLimitSwitchClosed) {
      // if (!climbRightEncoderZeroed) {
      //   climberRight.set(ControlMode.PercentOutput, climbRetractSlow.currentValue());
      // }
    } else {
      climberRight.setSelectedSensorPosition(0);
    }

    // soft limit, stop the motors if we are extending and pass our soft limit
    if (!climberTilted && climberLeft.getSelectedSensorPosition() >= (fullExtendPositionUprightLeft - 1000) && climberExtending) {
      climberLeft.set(TalonSRXControlMode.MotionMagic, fullExtendPositionUprightRight);
    }
    if (!climberTilted && climberRight.getSelectedSensorPosition() >= (fullExtendPositionUprightRight - 1000) && climberExtending) {
      climberRight.set(TalonSRXControlMode.MotionMagic, fullExtendPositionUprightRight);
    }

    if (!autoClimb) return;

    ClimbState nextState = ClimbState.Idle;

    switch (currentClimbState) {
      case Idle:
        if (autoClimbPressed) {
          nextState = ClimbState.ExtendPartial;
          extendPartialInit();
        } else {
          nextState = ClimbState.Idle;
        }
        break;
      case ExtendPartial:
        if (isClimberAtSetpoint(partialExtendPosition)) {
          nextState = ClimbState.ExtendFull;
          extendFullInit();
        } else {
          nextState = ClimbState.ExtendPartial;
        }
        break;
      case ExtendFull:
        if (isClimberAtSetpoint(fullExtendPositionUprightRight)) {
          nextState = ClimbState.Retract;
          retractInit();
        } else {
          nextState = ClimbState.ExtendFull;
        }
        break;
      case Retract:
        if (isClimberAtSetpoint(fullRetractPosition)) {
          nextState = ClimbState.Idle;
          idleInit();
        } else {
          nextState = ClimbState.Retract;
        }
        break;
    }

    // climberRight.follow(climberLeft, FollowerType.AuxOutput1);

    currentClimbState = nextState;
  }

  @Override
  public void disable() {
    climberLeft.set(0);
    climberRight.set(0);
  }

  public void manualElevatorExtend() {
    if (autoClimb) return;

    // TODO: LIMIT SWITCHES https://docs.ctre-phoenix.com/en/stable/ch13_MC.html#limit-switches
    // TODO: you should have the joystick/ button move the motion magic setpoint, not the motor in PWM mode
    if (climberTilted || climberLeft.getSelectedSensorPosition() < fullExtendPositionUprightLeft) {
      climberLeft.set(ControlMode.PercentOutput, climbOutput.currentValue());
      climberExtending = true;
    }
    // climberRight.follow(climberLeft, FollowerType.AuxOutput1);
    if (climberTilted || climberRight.getSelectedSensorPosition() < fullExtendPositionUprightRight) {
      climberRight.set(ControlMode.PercentOutput, climbOutput.currentValue());
      climberExtending = true;
    }

    climbState.log(LogLevel.GENERAL, "elevatorExtend");
  }

  public void manualElevatorRetract() {
    if (autoClimb) return;

    climberExtending = false;

    // TODO: LIMIT SWITCHES https://docs.ctre-phoenix.com/en/stable/ch13_MC.html#limit-switches
    // TODO: you should have the joystick/ button move the motion magic setpoint, not the motor in PWM mode
    climberLeft.set(ControlMode.PercentOutput, -climbOutput.currentValue());
    // climberRight.follow(climberLeft, FollowerType.AuxOutput1);
    climberRight.set(ControlMode.PercentOutput, -climbOutput.currentValue());

    climbState.log(LogLevel.GENERAL, "elevatorRetract");
  }

  public void elevatorStop() {

    climberLeft.set(0);
    climberRight.set(0);
    climberExtending = false;
    climbState.log(LogLevel.GENERAL, "climbStopped");
  }

  // autoClimb() both enables autoclimb and sets the flag to true
  // autoClimbReleased() just sets the flag to false
  public void autoClimb() {
    if (!config.enableClimberSubsystem) return;
    if (autoClimbStopped) return;

    autoClimb = true;
    autoClimbPressed = true;

    climbState.log(LogLevel.GENERAL, currentClimbState.toString());
  }

  private void autoExtendPartial() {
    // TODO: add a secondary control loop synchronizing the positions: docs.ctre-phoenix.com/en/stable/ch16_ClosedLoop.html#example-2-lift-mechanism
    // This can be accomplished by using the sum of each side as the elevator height, and the difference as the level deviation between the left and right, which must be kept near zero.
    // Aux PID[1] can then be used to apply a corrective difference component (adding to one side and subtracting from the other) to maintain a synchronous left and right position, while employing Position/Velocity/Motion-Magic to the primary axis of control (the elevator height).

    // try disabling the main PID loop
    // And check what the value of the error signal is
    // TODO: currently possibly you're applying feedback the wrong way

    climberExtending = true;

    // climberLeft.set(TalonSRXControlMode.MotionMagic, partialExtendPosition, DemandType.AuxPID, 0);
    climberLeft.set(TalonSRXControlMode.MotionMagic, partialExtendPosition);
    // climberRight.follow(climberLeft, FollowerType.AuxOutput1);
    climberRight.set(TalonSRXControlMode.MotionMagic, partialExtendPosition);
    // climberRight.set(TalonSRXControlMode.MotionMagic, partialExtendPosition, DemandType.AuxPID, 0);
  }

  private void autoExtendFull() {
    climberExtending = true;

    // climberLeft.set(TalonSRXControlMode.MotionMagic, fullExtendPosition, DemandType.AuxPID, 0);
    climberLeft.set(TalonSRXControlMode.MotionMagic, fullExtendPositionUprightRight);
    climberRight.set(TalonSRXControlMode.MotionMagic, fullExtendPositionUprightRight);
    // climberRight.follow(climberLeft, FollowerType.AuxOutput1);
    // climberRight.set(TalonSRXControlMode.MotionMagic, fullExtendPosition, DemandType.AuxPID, 0);
  }

  private void autoRetractFull() {
    climberExtending = false;
    // climberLeft.set(TalonSRXControlMode.MotionMagic, fullRetractPosition, DemandType.AuxPID, 0);
    climberLeft.set(TalonSRXControlMode.MotionMagic, fullRetractPosition);
    climberRight.set(TalonSRXControlMode.MotionMagic, fullRetractPosition);
    // climberRight.follow(climberLeft, FollowerType.AuxOutput1);
    // climberRight.set(TalonSRXControlMode.MotionMagic, fullRetractPosition, DemandType.AuxPID, 0);
  }

  public void stopAutoClimb() {
    climberExtending = false;
    autoClimb = false;
    autoClimbStopped = true;
    currentClimbState = ClimbState.Idle;
    disable();
  }

  public void setElevatorTilted(boolean tilted) {
    if (!config.enablePneumatics) return;

    climberTilted = tilted;
    elevatorTiltedState.log(LogLevel.GENERAL, false);

    if (tilted) {
      elevatorSolenoid.set(Value.kReverse);
    } else {
      elevatorSolenoid.set(Value.kForward);
    }
  }

  // TODO: (ideally) get rid of this
  public void elevatorToggle() {
    setElevatorTilted(!climberTilted);
  }

  public void resetClimbStuff() {
    autoClimbPressed = false;
    climberTilted = false;
    withinThresholdLoops1 = 0;
    withinThresholdLoops2 = 0;

    climberLeft.set(ControlMode.MotionMagic, 0);
    climberRight.set(ControlMode.MotionMagic, 0);

    climberLeft.getSensorCollection().setQuadraturePosition(0, MotorUtils.CONTROLLER_TIMEOUT_MS);
    climberRight.getSensorCollection().setQuadraturePosition(0, MotorUtils.CONTROLLER_TIMEOUT_MS);

    currentClimbState = ClimbState.Idle;
  }
}
