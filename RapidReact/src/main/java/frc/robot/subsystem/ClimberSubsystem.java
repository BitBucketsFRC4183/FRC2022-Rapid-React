package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Robot;
import frc.robot.config.Config;
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

  WPI_TalonSRX climber1; // leader
  WPI_TalonSRX climber2; // follower
  private boolean climberEnabled = true;

  private boolean autoClimb; // is autoclimb enabled
  private boolean autoClimbPressed = false; // is the autoclimb button currently being pressed

  DoubleSolenoid elevatorSolenoid;

  private int fullExtendPosition = 5000; // TODO: change this number
  private int partialExtendPosition = fullExtendPosition / 2;
  private int fullRetractPosition = 0; // TODO: change this number

  // https://docs.ctre-phoenix.com/en/stable/ch16_ClosedLoop.html#mechanism-is-finished-command
  private int climbErrThreshold = 10;
  private int climbLoopsToSettle = 10;
  private int withinThresholdLoops1 = 0;
  private int withinThresholdLoops2 = 0;

  private boolean climberTilted = false;

  private final Changeable<Double> climbOutput = BucketLog.changeable(Put.DOUBLE, "climber/climbOutput", 0.5);

  private final Loggable<String> climbState = BucketLog.loggable(Put.STRING, "climber/climbState");
  private final Loggable<Boolean> elevatorTiltedState = BucketLog.loggable(Put.BOOL, "climber/elevatorTiltedState");

  private final Loggable<Double> climber1Error = BucketLog.loggable(Put.DOUBLE, "climber/climber1Error");
  private final Loggable<Double> climber1Position = BucketLog.loggable(Put.DOUBLE, "climber/climber1Position");
  private final Loggable<Double> climber1Voltage = BucketLog.loggable(Put.DOUBLE, "climber/climber1Voltage");

  private final Loggable<Double> climber2Error = BucketLog.loggable(Put.DOUBLE, "climber/climber2Error");
  private final Loggable<Double> climber2Position = BucketLog.loggable(Put.DOUBLE, "climber/climber2Position");
  private final Loggable<Double> climber2Voltage = BucketLog.loggable(Put.DOUBLE, "climber/climber2Position");

  public ClimberSubsystem(Config config) {
    super(config);
  }

  @Override
  public void init() {
    climber1 = MotorUtils.makeSRX(config.climber.climber1);
    climber2 = MotorUtils.makeSRX(config.climber.climber2);

    if (config.enablePneumatics) {
      elevatorSolenoid =
        new DoubleSolenoid(PneumaticsModuleType.REVPH, config.elevatorSolenoid_ID1, config.elevatorSolenoid_ID2);
    }

    if (Robot.isSimulation()) {
      // simulate the motors
      CTREPhysicsSim.getInstance().addTalonSRX(climber1, .75, 5100, false);
      CTREPhysicsSim.getInstance().addTalonSRX(climber2, .75, 5100, false);
    }

    currentClimbState = ClimbState.Idle;
  }

  // TODO: properly test this
  boolean isClimberAtSetpoint(double setpoint) 
  {
    boolean climber1AtSetpoint = false;

    // if it's within the error threshold for a set amount of loops
    if (climber1.getClosedLoopError() < +climbErrThreshold && climber1.getClosedLoopError() > -climbErrThreshold) {
      withinThresholdLoops1++;
      if (withinThresholdLoops1 > climbLoopsToSettle) {
        // and ifs within the error threshold of the position
        if (
          climber1.getSelectedSensorPosition() > setpoint - climbErrThreshold &&
          climber1.getSelectedSensorPosition() < setpoint + climbErrThreshold
        ) {
          // then it's reached
          climber1AtSetpoint = true;
        }
      }
    } else {
      withinThresholdLoops1 = 0;
    }

    boolean climber2AtSetpoint = false;

    if (climber2.getClosedLoopError() < +climbErrThreshold && climber2.getClosedLoopError() > -climbErrThreshold) {
      withinThresholdLoops2++;
      if (withinThresholdLoops2 > climbLoopsToSettle) {
        if (
          climber2.getActiveTrajectoryPosition() > setpoint - climbErrThreshold &&
          climber2.getActiveTrajectoryPosition() < setpoint + climbErrThreshold
        ) {
          climber2AtSetpoint = true;
        }
      }
    } else {
      withinThresholdLoops2 = 0;
    }

    return climber1AtSetpoint && climber2AtSetpoint;
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
    climber1Error.log(LogLevel.GENERAL, climber1.getClosedLoopError());
    climber1Position.log(LogLevel.GENERAL, climber1.getSelectedSensorPosition());

    climber2Error.log(LogLevel.GENERAL, climber2.getClosedLoopError());
    climber2Position.log(LogLevel.GENERAL, climber2.getSelectedSensorPosition());

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

    climbState.log(LogLevel.GENERAL, currentClimbState.toString());

    climber1Error.log(LogLevel.GENERAL, climber1.getClosedLoopError());
    climber1Position.log(LogLevel.GENERAL, climber1.getSelectedSensorPosition());
    // Shuffleboard.getTab("SmartDashboard").addNumber("climber leader voltage", climber1.getSimCollection().getMotorOutputLeadVoltage());

    climber2Error.log(LogLevel.GENERAL, climber2.getClosedLoopError());
    climber2Position.log(LogLevel.GENERAL, climber2.getSelectedSensorPosition());
    climber2Voltage.log(LogLevel.GENERAL, climber2.getSimCollection().getMotorOutputLeadVoltage());

    currentClimbState = nextState;
  }

  @Override
  public void disable() {
    climber1.set(0);
    climber2.set(0);
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
    climber1.set(ControlMode.PercentOutput, climbOutput.currentValue());
    climber2.set(ControlMode.PercentOutput, climbOutput.currentValue());
  }

  public void manualElevatorRetract() { //uses down button
    if (autoClimb) return;

    // TODO: LIMIT SWITCHES https://docs.ctre-phoenix.com/en/stable/ch13_MC.html#limit-switches
    // TODO: you should have the joystick/ button move the motion magic setpoint, not the motor in PWM mode
    climber1.set(ControlMode.PercentOutput, -climbOutput.currentValue());
    climber2.set(ControlMode.PercentOutput, -climbOutput.currentValue());
  }

  public void elevatorStop() {
    climber1.set(0);
    climber2.set(0);
  }

  // autoClimb() both enables autoclimb and sets the flag to true
  // autoClimbReleased() just sets the flag to false
  public void autoClimb() { //uses TPAD button
    if (!config.enableClimberSubsystem) return;

    autoClimb = true;
    autoClimbPressed = true;
  }

  public void autoClimbReleased() {
    autoClimbPressed = false;
  }

  private void autoExtendPartial() {
    // TODO: (maybe do this in the tuning place?) add a secondary control loop synchronizing the positions: docs.ctre-phoenix.com/en/stable/ch16_ClosedLoop.html#example-2-lift-mechanism
    // This can be accomplished by using the sum of each side as the elevator height, and the difference as the level deviation between the left and right, which must be kept near zero.

    // Aux PID[1] can then be used to apply a corrective difference component (adding to one side and subtracting from the other) to maintain a synchronous left and right position, while employing Position/Velocity/Motion-Magic to the primary axis of control (the elevator height).
    climber1.set(TalonSRXControlMode.MotionMagic, partialExtendPosition);
    climber2.set(TalonSRXControlMode.MotionMagic, partialExtendPosition);
  }

  private void autoExtendFull() {
    climber1.set(TalonSRXControlMode.MotionMagic, fullExtendPosition);
    climber2.set(TalonSRXControlMode.MotionMagic, fullExtendPosition);
  }

  private void autoRetractFull() {
    climber1.set(TalonSRXControlMode.MotionMagic, fullRetractPosition);
    climber2.set(TalonSRXControlMode.MotionMagic, fullRetractPosition);
  }

  // TODO: add a button for this
  private void stopAutoClimb() {
    autoClimb = false;
    currentClimbState = ClimbState.Idle;
  }

  public void setElevatorTilted(boolean tilted) {
    if (!config.enablePneumatics) return;

    climberTilted = tilted;
    elevatorTiltedState.log(LogLevel.GENERAL, climberTilted);

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
