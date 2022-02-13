package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.config.Config;
import frc.robot.log.*;
import frc.robot.utils.MotorUtils;

public class ClimberSubsystem extends BitBucketsSubsystem {

  // state machine diagram 
  // https://i.imgur.com/zE1pEXn.png
  enum ClimbState
  {
    Idle,
    ExtendPartial,
    ExtendFull,
    Retract
  }

  private WPI_TalonSRX climber1;
  private WPI_TalonSRX climber2;
  private boolean enabledClimber = true;

  private boolean autoClimb; // is autoclimb enabled
  private boolean autoClimbPressed = false; // is the autoclimb button currently being pressed

  private ClimbState currentClimbState;
  DoubleSolenoid elevatorSolenoid;
  
  private int fullExtendPosition = 5000; // TODO: change this number
  private int partialExtendPosition = fullExtendPosition / 2;
  private int fullRetractPosition = 0; // TODO: change this number

  // https://docs.ctre-phoenix.com/en/stable/ch16_ClosedLoop.html#mechanism-is-finished-command
  private int climbErrThreshold1;
  private int climbLoopsToSettle1;
  private int withinThresholdLoops1;

  private int climbErrThreshold2;
  private int climbLoopsToSettle2;
  private int withinThresholdLoops2;

  private boolean climberTilted = false;

  private final Changeable<Double> climbOutput = BucketLog.changeable(Put.DOUBLE, "climber/climbOutput", 0.5);

  private final Loggable<String> climbState = BucketLog.loggable(Put.STRING, "climber/climbState");
  private final Loggable<String> elevatorTiltedState = BucketLog.loggable(Put.STRING, "climber/elevatorTiltedState");
  private final Loggable<Double> climberMotorPosition = BucketLog.loggable(Put.DOUBLE, "climber/climberMotorPosition");

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

    currentClimbState = ClimbState.Idle;
  }

  boolean isClimberAtSetpoint()
  {
    climbErrThreshold1 = 10;
    climbLoopsToSettle1 = 10;
    withinThresholdLoops1 = 0;
    boolean climber1AtSetpoint = false;
    if (climber1.getClosedLoopError() < +climbErrThreshold1 && climber1.getClosedLoopError() > -climbErrThreshold1) {
      withinThresholdLoops1++;

      if (withinThresholdLoops1 > climbLoopsToSettle1)
        climber1AtSetpoint = true;
    } else {
      withinThresholdLoops1 = 0;
    }
    
    climbErrThreshold2 = 10;
    climbLoopsToSettle2 = 10;
    withinThresholdLoops2 = 0;
    boolean climber2AtSetpoint = false;
    if (climber2.getClosedLoopError() < +climbErrThreshold2 && climber2.getClosedLoopError() > -climbErrThreshold2) {
      withinThresholdLoops2++;

      if (withinThresholdLoops2 > climbLoopsToSettle2)
        climber2AtSetpoint = true;
    } else {
      withinThresholdLoops2 = 0;
    }

    return climber1AtSetpoint && climber2AtSetpoint;
  }

  private void idleInit()
  {
    setElevatorTilted(false);
    autoRetractFull();
  }

  private void extendPartialInit()
  {
    setElevatorTilted(false);
    autoExtendPartial();
  }

  private void extendFullInit()
  {
    setElevatorTilted(true);
    autoExtendFull();
  }

  private void retractInit()
  {
    setElevatorTilted(false);
    autoRetractFull();
  }

  @Override
  public void periodic() {
    if (!autoClimb)
      return;
    
    ClimbState nextState = ClimbState.Idle;

    switch (currentClimbState)
    {
      case Idle:
        if (autoClimbPressed)
        {
          nextState = ClimbState.ExtendPartial;
          extendPartialInit();
        }
        else
        {
          nextState = ClimbState.Idle;
        }
        break;
    
      case ExtendPartial:
        if (isClimberAtSetpoint())
        {
          nextState = ClimbState.ExtendFull;
          extendFullInit();
        }
        else
        {
          nextState = ClimbState.ExtendPartial;
        }
        break;
      
      case ExtendFull:
        if (isClimberAtSetpoint())
        {
          nextState = ClimbState.Retract;
          retractInit();
        }
        else
        {
          nextState = ClimbState.ExtendFull;
        }
        break;
      
      case Retract:
        if (isClimberAtSetpoint())
        {
          nextState = ClimbState.Idle;
          idleInit();
        }
        else
        {
          nextState = ClimbState.Retract;      
        }
        break;
    }

    currentClimbState = nextState;
  }


  @Override
  public void disable() {
    climber1.set(0);
    climber2.set(0);
  }

  public void toggleClimberEnabled() { // uses 2 PS button
    enabledClimber = !enabledClimber;

    if (enabledClimber) {
      climbState.log(LogLevel.GENERAL, "climberEnabled");
    } else {
      climbState.log(LogLevel.GENERAL, "climberDisabled");
    }
  }

  public void manualElevatorExtend() { //uses up button
    if (autoClimb) return;

    // TODO PERSISTNET: LIMIT SWITCHES https://docs.ctre-phoenix.com/en/stable/ch13_MC.html#limit-switches
    // TODO PERSISTENT: you should have the joystick/ button move the motion magic setpoint, not the motor in PWM mode

    climber1.set(ControlMode.PercentOutput, climbOutput.currentValue());
    climber2.set(ControlMode.PercentOutput, climbOutput.currentValue());
    climberMotorPosition.log(climber1.getSelectedSensorPosition());

    climbState.log(LogLevel.GENERAL, "elevatorExtend");
  }

  public void manualElevatorRetract() { //uses down button
    if (autoClimb) return;

    // TODO PERSISTNET: LIMIT SWITCHES https://docs.ctre-phoenix.com/en/stable/ch13_MC.html#limit-switches
    // TODO PERSISTENT: you should have the joystick/ button move the motion magic setpoint, not the motor in PWM mode

    climber1.set(ControlMode.PercentOutput, -climbOutput.currentValue());
    climber2.set(ControlMode.PercentOutput, -climbOutput.currentValue());
    climberMotorPosition.log(climber1.getSelectedSensorPosition());

    climbState.log(LogLevel.GENERAL, "elevatorRetract");
  }

  public void elevatorStop() {
    climber1.set(0);
    climber2.set(0);
    climbState.log(LogLevel.GENERAL, "climbStopped");
  }

  // autoClimb() both enables autoclimb and sets the flag to true
  // autoClimbReleased() just sets the flag to false
  public void autoClimb() { //uses TPAD button
    autoClimb = true;
    autoClimbPressed = true;

    climbState.log(LogLevel.GENERAL, "automatically climbing");
  }
  public void autoClimbReleased()
  {
    autoClimbPressed = false;
  }

  private void autoExtendPartial()
  {
    climber1.set(TalonSRXControlMode.MotionMagic, partialExtendPosition);
    climber2.set(TalonSRXControlMode.MotionMagic, partialExtendPosition);

    climberMotorPosition.log(climber1.getSelectedSensorPosition());
    climbState.log(LogLevel.GENERAL, "elevatorExtendPartial");
  }

  private void autoExtendFull()
  {
    climber1.set(TalonSRXControlMode.MotionMagic, fullExtendPosition);
    climber2.set(TalonSRXControlMode.MotionMagic, fullExtendPosition);

    climberMotorPosition.log(climber1.getSelectedSensorPosition());
    climbState.log(LogLevel.GENERAL, "elevatorExtendFull");
  }

  private void autoRetractFull()
  {
    climber1.set(TalonSRXControlMode.MotionMagic, fullRetractPosition);
    climber2.set(TalonSRXControlMode.MotionMagic, fullRetractPosition);

    climberMotorPosition.log(climber1.getSelectedSensorPosition());
    climbState.log(LogLevel.GENERAL, "elevatorRetract");
  }

  // TODO: add a button for this
  private void stopAutoClimb() {
    autoClimb = false;
    currentClimbState = ClimbState.Idle;
  }

  public void setElevatorTilted(boolean tilted)
  {
    if (!config.enablePneumatics)
      return;
    
    climberTilted = tilted;
    
    if (tilted)
    {
      elevatorSolenoid.set(Value.kReverse);
      elevatorTiltedState.log(LogLevel.GENERAL, "Tilted");
    }
    else
    {
      elevatorSolenoid.set(Value.kForward);
      elevatorTiltedState.log(LogLevel.GENERAL, "Not Tilted");
    }
  }

  public void elevatorToggle()
  {
    setElevatorTilted(!climberTilted);
  }

  public boolean getEnabledClimber() {
    return enabledClimber;
  }
}
