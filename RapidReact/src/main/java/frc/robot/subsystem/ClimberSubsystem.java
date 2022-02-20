package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
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

  WPI_TalonSRX climberLeader = new WPI_TalonSRX(config.climberMotor_IDLeader); // leader
  WPI_TalonSRX climberFollower = new WPI_TalonSRX(config.climberMotor_IDFollower); // follower
  MotorConfig leaderConfig = config.climber.climberLeader;
  MotorConfig followerConfig = config.climber.climberFollower;

  private boolean climberEnabled = true;

  private boolean autoClimb; // is autoclimb enabled
  private boolean autoClimbPressed = false; // is the autoclimb button currently being pressed

  DoubleSolenoid elevatorSolenoid;

  private int fullExtendPosition = 500000; // TODO: change this number
  private int partialExtendPosition = fullExtendPosition / 2;
  private int fullRetractPosition = 0; // TODO: change this number

  // https://docs.ctre-phoenix.com/en/stable/ch16_ClosedLoop.html#mechanism-is-finished-command
  private int climbErrThreshold = 1000;
  private int climbLoopsToSettle = 10;
  private int withinThresholdLoops1 = 0;
  private int withinThresholdLoops2 = 0;

  private boolean climberTilted = false;

  private final Changeable<Double> climbOutput = BucketLog.changeable(Put.DOUBLE, "climber/climbOutput", 0.5);

  private final Loggable<String> climbState = BucketLog.loggable(Put.STRING, "climber/climbState");
  private final Loggable<Boolean> elevatorTiltedState = BucketLog.loggable(Put.BOOL, "climber/elevatorTiltedState");

  private final Loggable<Double> climberLeaderError = BucketLog.loggable(Put.DOUBLE, "climber/climberLeaderError");
  private final Loggable<Double> climberLeaderPosition = BucketLog.loggable(
    Put.DOUBLE,
    "climber/climberLeaderPosition"
  );
  // private final Loggable<Double> climberLeaderVoltage = BucketLog.loggable(Put.DOUBLE, "climber/climberLeaderPosition");
  private final Loggable<Double> climberLeaderVelocity = BucketLog.loggable(Put.DOUBLE, "climber/climberLeaderVelocity");

  private final Loggable<Double> climberFollowerError = BucketLog.loggable(Put.DOUBLE, "climber/climberFollowerError");
  private final Loggable<Double> climberFollowerPosition = BucketLog.loggable(
    Put.DOUBLE,
    "climber/climberFollowerPosition"
  );

  // private final Loggable<Double> climberFollowerVoltage = BucketLog.loggable(Put.DOUBLE, "climber/climberFollowerPosition");

  public ClimberSubsystem(Config config) {
    super(config);
  }

  @Override
  public void init() {
    climberLeader.set(ControlMode.MotionMagic, 0);
    climberFollower.set(ControlMode.MotionMagic, 0);

    climberLeader.configFactoryDefault();
    climberFollower.configFactoryDefault();

    climberLeader.setNeutralMode(NeutralMode.Brake);
    climberLeader.setNeutralMode(NeutralMode.Brake);

    // climberLeader = MotorUtils.makeSRX(leaderConfig);
    // climberFollower = MotorUtils.makeSRX(config.climber.climberFollower);
    // climberFollower.follow(climberLeader, FollowerType.AuxOutput1);

    // Configure the left Talon's selected sensor as local QuadEncoder
    climberLeader.configSelectedFeedbackSensor(
      FeedbackDevice.QuadEncoder,
      MotorUtils.PRIMARY_PID_LOOP,
      MotorUtils.CONTROLLER_TIMEOUT_MS
    );
    // Configure the Remote Talon's selected sensor as a remote sensor for the right Talon
    // climberFollower.configRemoteFeedbackFilter(
    //   climberLeader.getDeviceID(),
    //   RemoteSensorSource.TalonSRX_SelectedSensor,
    //   MotorUtils.REMOTE_0
    // );
    climberFollower.configSelectedFeedbackSensor(
      FeedbackDevice.QuadEncoder,
      MotorUtils.PRIMARY_PID_LOOP,
      MotorUtils.CONTROLLER_TIMEOUT_MS
    );

    // Setup Sum signal to be used for Distance
    // climberFollower.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor0);
    // climberFollower.configSensorTerm(SensorTerm.Sum1, FeedbackDevice.CTRE_MagEncoder_Relative);

    // Setup Difference signal to be used for Turn
    // climberFollower.configSensorTerm(SensorTerm.Diff0, FeedbackDevice.RemoteSensor0);
    // climberFollower.configSensorTerm(SensorTerm.Diff1, FeedbackDevice.CTRE_MagEncoder_Relative);

    // Configure Sum [Sum of both QuadEncoders] to be used for Primary PID Index
    // climberFollower.configSelectedFeedbackSensor(FeedbackDevice.SensorSum, MotorUtils.PRIMARY_PID_LOOP, 0);
    // Scale Feedback by 0.5 to half the sum of Distance
    // climberFollower.configSelectedFeedbackCoefficient(
    //   0.5,
    //   MotorUtils.PRIMARY_PID_LOOP,
    //   MotorUtils.CONTROLLER_TIMEOUT_MS
    // );

    // Configure Difference [Difference between both QuadEncoders] to be used for Auxiliary PID Index
    // climberFollower.configSelectedFeedbackSensor(
    //   FeedbackDevice.SensorDifference,
    //   MotorUtils.PRIMARY_PID_LOOP,
    //   MotorUtils.CONTROLLER_TIMEOUT_MS
    // );
    // Scale the Feedback Sensor using a coefficient
    // climberFollower.configSelectedFeedbackCoefficient(1, MotorUtils.TURN_PID_LOOP, MotorUtils.CONTROLLER_TIMEOUT_MS);

    // Configure output and sensor direction
    climberLeader.setInverted(leaderConfig.inverted);
    climberLeader.setSensorPhase(leaderConfig.sensorPhase);
    climberFollower.setInverted(followerConfig.inverted);
    climberFollower.setSensorPhase(followerConfig.sensorPhase);

    // Set status frame periods to ensure we don't have stale data
    climberLeader.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20);
    climberLeader.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20);
    climberLeader.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20);
    climberLeader.setStatusFramePeriod(StatusFrame.Status_10_Targets, 20);
    climberFollower.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 5);
    climberFollower.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20);
    climberFollower.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20);
    climberFollower.setStatusFramePeriod(StatusFrame.Status_10_Targets, 20);

    // Configure neutral deadband
    climberLeader.configNeutralDeadband(MotorUtils.kNeutralDeadband);
    climberFollower.configNeutralDeadband(MotorUtils.kNeutralDeadband);

    // Motion Magic Configurations
    climberLeader.configMotionAcceleration(leaderConfig.motionMagicAcceleration);
    climberLeader.configMotionCruiseVelocity(leaderConfig.motionMagicCruiseVelocity);
    climberFollower.configMotionAcceleration(followerConfig.motionMagicAcceleration);
    climberFollower.configMotionCruiseVelocity(followerConfig.motionMagicCruiseVelocity);

    /**
     * Max out the peak output (for all modes).
     * However you can limit the output of a given PID object with configClosedLoopPeakOutput().
     */
    climberLeader.configPeakOutputForward(+1.0);
    climberLeader.configPeakOutputReverse(-1.0);
    climberFollower.configPeakOutputForward(+1.0);
    climberFollower.configPeakOutputReverse(-1.0);

    /* FPID Gains for distance servo */
    climberLeader.config_kP(MotorUtils.positionSlot, leaderConfig.positionPIDF.getKP());
    climberLeader.config_kI(MotorUtils.positionSlot, leaderConfig.positionPIDF.getKI());
    climberLeader.config_kD(MotorUtils.positionSlot, leaderConfig.positionPIDF.getKD());
    climberLeader.config_kF(MotorUtils.positionSlot, leaderConfig.positionPIDF.getKF());
    climberLeader.config_IntegralZone(MotorUtils.positionSlot, leaderConfig.positionPIDF.getIZone());
    climberLeader.configClosedLoopPeakOutput(MotorUtils.positionSlot, leaderConfig.distancePeakOutput);
    climberLeader.configAllowableClosedloopError(MotorUtils.positionSlot, 0);

    climberFollower.config_kP(MotorUtils.positionSlot, followerConfig.positionPIDF.getKP());
    climberFollower.config_kI(MotorUtils.positionSlot, followerConfig.positionPIDF.getKI());
    climberFollower.config_kD(MotorUtils.positionSlot, followerConfig.positionPIDF.getKD());
    climberFollower.config_kF(MotorUtils.positionSlot, followerConfig.positionPIDF.getKF());
    climberFollower.config_IntegralZone(MotorUtils.positionSlot, followerConfig.positionPIDF.getIZone());
    climberFollower.configClosedLoopPeakOutput(MotorUtils.positionSlot, followerConfig.distancePeakOutput);
    climberFollower.configAllowableClosedloopError(MotorUtils.positionSlot, 0);

    /* FPID Gains for turn servo */
    // climberLeader.config_kP(MotorUtils.velocitySlot, leaderConfig.positionPIDF.getKP());
    // climberLeader.config_kI(MotorUtils.velocitySlot, leaderConfig.positionPIDF.getKI());
    // climberLeader.config_kD(MotorUtils.velocitySlot, leaderConfig.positionPIDF.getKD());
    // climberLeader.config_kF(MotorUtils.velocitySlot, leaderConfig.positionPIDF.getKF());
    // climberLeader.config_IntegralZone(MotorUtils.velocitySlot, leaderConfig.positionPIDF.getIZone());
    // climberFollower.configClosedLoopPeakOutput(MotorUtils.velocitySlot, leaderConfig.turningPeakOutput);
    // climberFollower.configAllowableClosedloopError(MotorUtils.velocitySlot, 0);

    /**
     * 1ms per loop.  PID loop can be slowed down if need be.
     * For example,
     * - if sensor updates are too slow
     * - sensor deltas are very small per update, so derivative error never gets large enough to be useful.
     * - sensor movement is very slow causing the derivative error to be near zero.
     */
    // int closedLoopTimeMs = 1;
    // climberFollower.configClosedLoopPeriod(0, closedLoopTimeMs);
    // climberFollower.configClosedLoopPeriod(1, closedLoopTimeMs);

    /**
     * configAuxPIDPolarity(boolean invert, int timeoutMs)
     * false means talon's local output is PID0 + PID1, and other side Talon is PID0 - PID1
     * true means talon's local output is PID0 - PID1, and other side Talon is PID0 + PID1
     */
    // climberFollower.configAuxPIDPolarity(false);

    /* Initialize */
    // climberFollower.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 10);

    // zero sensors
    climberLeader.getSensorCollection().setQuadraturePosition(0, MotorUtils.CONTROLLER_TIMEOUT_MS);
    climberFollower.getSensorCollection().setQuadraturePosition(0, MotorUtils.CONTROLLER_TIMEOUT_MS);

    if (config.enablePneumatics) {
      elevatorSolenoid =
        new DoubleSolenoid(PneumaticsModuleType.REVPH, config.elevatorSolenoid_ID1, config.elevatorSolenoid_ID2);
    }

    if (Robot.isSimulation()) {
      // simulate the motors
      CTREPhysicsSim.getInstance().addTalonSRX(climberLeader, .75, 5100, false);
      CTREPhysicsSim.getInstance().addTalonSRX(climberFollower, .75, 5100, false);
    }

    currentClimbState = ClimbState.Idle;
  }

  boolean isClimberAtSetpoint(double setpoint) {
    boolean climberLeaderAtSetpoint = false;

    // if it's within the error threshold for a set amount of loops
    if (
      climberLeader.getClosedLoopError() < +climbErrThreshold && climberLeader.getClosedLoopError() > -climbErrThreshold
    ) {
      withinThresholdLoops1++;
      if (withinThresholdLoops1 > climbLoopsToSettle) {
        // and ifs within the error threshold of the position
        if (
          climberLeader.getSelectedSensorPosition() > setpoint - climbErrThreshold &&
          climberLeader.getSelectedSensorPosition() < setpoint + climbErrThreshold
        ) {
          // then it's reached
          climberLeaderAtSetpoint = true;
        }
      }
    } else {
      withinThresholdLoops1 = 0;
    }

    boolean climberFollowerAtSetpoint = false;

    if (
      climberFollower.getClosedLoopError() < +climbErrThreshold &&
      climberFollower.getClosedLoopError() > -climbErrThreshold
    ) {
      withinThresholdLoops2++;
      if (withinThresholdLoops2 > climbLoopsToSettle) {
        if (
          climberFollower.getSelectedSensorPosition() > setpoint - climbErrThreshold &&
          climberFollower.getSelectedSensorPosition() < setpoint + climbErrThreshold
        ) {
          climberFollowerAtSetpoint = true;
        }
      }
    } else {
      withinThresholdLoops2 = 0;
    }

    return climberLeaderAtSetpoint && climberFollowerAtSetpoint;
  }

  private void idleInit() {
    // setElevatorTilted(false);
    autoRetractFull();
  }

  private void extendPartialInit() {
    // setElevatorTilted(false);
    autoExtendPartial();
  }

  private void extendFullInit() {
    // setElevatorTilted(true);
    autoExtendFull();
  }

  private void retractInit() {
    // TODO: ensure the hooks have engaged the bar before we begin retracting?
    // setElevatorTilted(false);
    autoRetractFull();
  }

  @Override
  public void periodic() {
    climbState.log(LogLevel.GENERAL, currentClimbState.toString());

    climberLeaderPosition.log(LogLevel.GENERAL, climberLeader.getSelectedSensorPosition());
    climberLeaderError.log(LogLevel.GENERAL, climberLeader.getClosedLoopError());
    climberLeaderVelocity.log(LogLevel.GENERAL, climberLeader.getSelectedSensorVelocity());

    climberFollowerPosition.log(LogLevel.GENERAL, climberFollower.getSelectedSensorPosition());
    climberFollowerError.log(LogLevel.GENERAL, climberFollower.getClosedLoopError());

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
        if (isClimberAtSetpoint(fullExtendPosition)) {
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

    // climberFollower.follow(climberLeader, FollowerType.AuxOutput1);

    currentClimbState = nextState;
  }

  @Override
  public void disable() {
    climberLeader.set(0);
    climberFollower.set(0);
  }

  public void toggleClimberEnabled() { // uses 2 PS button
    climberEnabled = !climberEnabled;

    if (climberEnabled) {
      climbState.log(LogLevel.GENERAL, "climberEnabled");
    } else {
      climbState.log(LogLevel.GENERAL, "climberDisabled");
    }
  }

  public void manualElevatorExtend() { //uses up button
    if (autoClimb) return;

    // TODO: LIMIT SWITCHES https://docs.ctre-phoenix.com/en/stable/ch13_MC.html#limit-switches
    // TODO: you should have the joystick/ button move the motion magic setpoint, not the motor in PWM mode
    climberLeader.set(ControlMode.PercentOutput, climbOutput.currentValue());
    // climberFollower.follow(climberLeader, FollowerType.AuxOutput1);
    climberFollower.set(ControlMode.PercentOutput, climbOutput.currentValue());

    climbState.log(LogLevel.GENERAL, "elevatorExtend");
  }

  public void manualElevatorRetract() { //uses down button
    if (autoClimb) return;

    // TODO: LIMIT SWITCHES https://docs.ctre-phoenix.com/en/stable/ch13_MC.html#limit-switches
    // TODO: you should have the joystick/ button move the motion magic setpoint, not the motor in PWM mode
    climberLeader.set(ControlMode.PercentOutput, -climbOutput.currentValue());
    // climberFollower.follow(climberLeader, FollowerType.AuxOutput1);
    climberFollower.set(ControlMode.PercentOutput, -climbOutput.currentValue());

    climbState.log(LogLevel.GENERAL, "elevatorRetract");
  }

  public void elevatorStop() {
    climberLeader.set(0);
    climberFollower.set(0);
    climbState.log(LogLevel.GENERAL, "climbStopped");
  }

  // autoClimb() both enables autoclimb and sets the flag to true
  // autoClimbReleased() just sets the flag to false
  public void autoClimb() { //uses TPAD button
    if (!config.enableClimberSubsystem) return;

    autoClimb = true;
    autoClimbPressed = true;

    climbState.log(LogLevel.GENERAL, currentClimbState.toString());
  }

  public void autoClimbReleased() {
    autoClimbPressed = false;
  }

  private void autoExtendPartial() {
    // TODO: (maybe do this in the tuning place?) add a secondary control loop synchronizing the positions: docs.ctre-phoenix.com/en/stable/ch16_ClosedLoop.html#example-2-lift-mechanism
    // This can be accomplished by using the sum of each side as the elevator height, and the difference as the level deviation between the left and right, which must be kept near zero.

    // Aux PID[1] can then be used to apply a corrective difference component (adding to one side and subtracting from the other) to maintain a synchronous left and right position, while employing Position/Velocity/Motion-Magic to the primary axis of control (the elevator height).

    // TODO: dif value for target turn that isn't 0? (for all of these 3 functions)
    // climberLeader.set(TalonSRXControlMode.MotionMagic, partialExtendPosition, DemandType.AuxPID, 0);
    climberLeader.set(TalonSRXControlMode.MotionMagic, partialExtendPosition);
    // climberFollower.follow(climberLeader, FollowerType.AuxOutput1);
    climberFollower.set(TalonSRXControlMode.MotionMagic, partialExtendPosition);
    // climberFollower.set(TalonSRXControlMode.MotionMagic, partialExtendPosition, DemandType.AuxPID, 0);
  }

  private void autoExtendFull() {
    // climberLeader.set(TalonSRXControlMode.MotionMagic, fullExtendPosition, DemandType.AuxPID, 0);
    climberLeader.set(TalonSRXControlMode.MotionMagic, fullExtendPosition);
    climberFollower.set(TalonSRXControlMode.MotionMagic, fullExtendPosition);
    // climberFollower.follow(climberLeader, FollowerType.AuxOutput1);
    // climberFollower.set(TalonSRXControlMode.MotionMagic, fullExtendPosition, DemandType.AuxPID, 0);
  }

  private void autoRetractFull() {
    // climberLeader.set(TalonSRXControlMode.MotionMagic, fullRetractPosition, DemandType.AuxPID, 0);
    climberLeader.set(TalonSRXControlMode.MotionMagic, fullRetractPosition);
    climberFollower.set(TalonSRXControlMode.MotionMagic, fullRetractPosition);
    // climberFollower.follow(climberLeader, FollowerType.AuxOutput1);
    // climberFollower.set(TalonSRXControlMode.MotionMagic, fullRetractPosition, DemandType.AuxPID, 0);
  }

  // TODO: add a button for this
  private void stopAutoClimb() {
    autoClimb = false;
    currentClimbState = ClimbState.Idle;
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

  public boolean isClimberEnabled() {
    return climberEnabled;
  }
}
