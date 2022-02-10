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

  enum ClimbPhase {
    Idle, // not climbing (initial value)
    ClimbMid, // climbing from ground to mid
    ClimbHigh, // climbing from mid to high
    ClimbTraversal, // climbing for high to traversal
  }

  ClimbPhase[] climbPhaseValues = ClimbPhase.values();

  enum ClimbState {
    Idle, // not climbing (initial value)
    Climbing,
    Finished,
    Stoppped,
  }

  enum ClimbAction {
    Idle,
    Extending,
    Retracting,
  }

  private WPI_TalonSRX climber1;
  private WPI_TalonSRX climber2;
  private boolean enabledClimber = true;
  private boolean elevatorToggle;
  private boolean autoClimb;
  private ClimbPhase currentClimbPhase;
  private ClimbState currentClimbState;
  private ClimbAction currentClimbAction;
  DoubleSolenoid elevatorSolenoid;
  
  private int fullExtendPosition = 5000; // TODO: change this number
  private int halfExtendPosition = fullExtendPosition / 2;
  private int fullRetractPosition = 0; // TODO: change this number

  private final Changeable<Double> climbOutput = BucketLog.changeable(Put.DOUBLE, "climber/climbOutput", 0.5);

  private final Loggable<String> climbState = BucketLog.loggable(Put.STRING, "climber/climbState");
  private final Loggable<String> elevatorToggleState = BucketLog.loggable(Put.STRING, "climber/elevatorState");
  private final Loggable<Double> climberMotorPosition = BucketLog.loggable(Put.DOUBLE, "climber/climberMotorPosition");

  public ClimberSubsystem(Config config) {
    super(config);
  }

  @Override
  public void init() {
    climber1 = MotorUtils.makeSRX(config.climber.climber1);
    climber2 = MotorUtils.makeSRX(config.climber.climber2);
    climber2.follow(climber1);

    if (config.enablePneumatics) {
      elevatorSolenoid =
        new DoubleSolenoid(PneumaticsModuleType.REVPH, config.elevatorSolenoid_ID1, config.elevatorSolenoid_ID2);
    }

    currentClimbPhase = ClimbPhase.Idle;
    currentClimbState = ClimbState.Idle;
    currentClimbAction = ClimbAction.Idle;
  }

  @Override
  public void periodic() {
    if (autoClimb) {
      if (currentClimbState == ClimbState.Stoppped) {}
      if (currentClimbState == ClimbState.Finished) {
        // ground -> mid
        // toggle, extend, stop at limit, toggle back, retract, stop at limit, end phase

        if (currentClimbAction == ClimbAction.Idle)
          startForwardAutoClimb();
        
        // anything except going from ground -> mid
        if (currentClimbPhase != ClimbPhase.ClimbMid)
        {
          // toggle arm back once we've gone halfway
          if (climber1.getSelectedSensorPosition() >= halfExtendPosition) {
            elevatorToggle();
          }
        }

        // "You make the winch rope short enough that it’s impossible for the climber to come completely out the top, then you look for a motor current spike to tell when it’s fully retracted and the motor is stalled"

        if (climber1.getSelectedSensorPosition() >= fullExtendPosition)
        {
          elevatorStop();
          currentClimbAction = ClimbAction.Idle;
        }

        // don't go past this point until finished extending
        if (currentClimbAction == ClimbAction.Extending) return;

        // only call while idle; not repreatedly whilst reversing
        if (currentClimbAction == ClimbAction.Idle) 
          startReverseAutoClimbHighTraversal();

        if (climber1.getSelectedSensorPosition() >= fullRetractPosition)
        {
          elevatorStop();
          currentClimbAction = ClimbAction.Idle;
          currentClimbState = ClimbState.Finished;
          // autoClimb = false;
        }
      }
    }
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

  public void elevatorExtend() { //uses up button
    if (autoClimb) return;

    climber1.set(ControlMode.PercentOutput, climbOutput.currentValue());
    climber2.set(ControlMode.PercentOutput, climbOutput.currentValue());
    climberMotorPosition.log(climber1.getSelectedSensorPosition());

    climbState.log(LogLevel.GENERAL, "elevatorExtend");
  }

  public void elevatorRetract() { //uses down button
    if (autoClimb) return;

    climber1.set(ControlMode.PercentOutput, -climbOutput.currentValue());
    climber2.set(ControlMode.PercentOutput, -climbOutput.currentValue());
    climberMotorPosition.log(climber1.getSelectedSensorPosition());

    climbState.log(LogLevel.GENERAL, "elevatorRetract");
  }

  public void elevatorStop() {
    climber1.set(0);
    climber2.set(0);
    climbState.log(LogLevel.GENERAL, "climbStotopped");
  }

  public void climbAuto() { //uses TPAD button
    autoClimb = true;
    climbState.log(LogLevel.GENERAL, "automatially climbing");
    // TODO: figure out behavior if it's stopped
    // my postulation is that we go to the next phase
    // i assume that controllers might stop it, finish the climb, then hit the button for the next phase
  }

  private void startForwardAutoClimb() {
    switch (currentClimbPhase) {
      case Idle:
        currentClimbPhase = ClimbPhase.ClimbMid;
        break;
      case ClimbMid:
        currentClimbPhase = ClimbPhase.ClimbHigh;
        break;
      case ClimbHigh:
        currentClimbPhase = ClimbPhase.ClimbTraversal;
        break;
    }

    elevatorToggle();

    currentClimbAction = ClimbAction.Extending;

    climber1.set(TalonSRXControlMode.MotionMagic, fullExtendPosition);
    climberMotorPosition.log(climber1.getSelectedSensorPosition());
    climbState.log(LogLevel.GENERAL, "elevatorExtend");
  }

  private void startReverseAutoClimbHighTraversal() {
    elevatorToggle();

    currentClimbAction = ClimbAction.Retracting;

    climber1.set(TalonSRXControlMode.MotionMagic, fullRetractPosition);
    climberMotorPosition.log(climber1.getSelectedSensorPosition());
    climbState.log(LogLevel.GENERAL, "elevatorExtend");
  }

  // TODO: add a button for this
  private void stopAutoClimb() {
    autoClimb = false;
    if (currentClimbState != ClimbState.Idle) {
      currentClimbState = ClimbState.Stoppped;
    }
  }

  public void elevatorToggle() {
    if (config.enablePneumatics) {
      elevatorToggle = !elevatorToggle;
      if (!elevatorToggle) {
        elevatorSolenoid.set(Value.kForward);
        elevatorToggleState.log(LogLevel.GENERAL, "ClimbTime");
      } else {
        elevatorSolenoid.set(Value.kReverse);
        elevatorToggleState.log(LogLevel.GENERAL, "NotClimbTime");
      }
    }
  }

  public boolean getEnabledClimber() {
    return enabledClimber;
  }
}
