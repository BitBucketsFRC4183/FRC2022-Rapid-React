package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.config.Config;
import frc.robot.log.*;
import javax.swing.plaf.basic.BasicButtonUI;

public class ClimberSubsystem extends BitBucketsSubsystem {

  enum ClimbPhase {
    Idle, // not climbing (initial value)
    ClimbMid, // climbing from ground to mid
    ClimbHigh, // climbing from mid to high
    ClimbTraversal, // climbing for high to traversal
  }

  enum ClimbState {
    Idle, // not climbing (initial value)
    Climbing,
    Finished,
    Stoppped,
  }

  private WPI_TalonSRX climber1;
  private WPI_TalonSRX climber2;
  private boolean enabledClimber = true;
  private boolean fixedHookToggleState;
  private boolean elevatorToggle;
  private boolean autoClimb;
  private ClimbPhase currentClimbPhase;
  private ClimbState currentClimbState;
  DoubleSolenoid elevatorSolenoid;

  private final Changeable<Double> climbOutput = BucketLog.changeable(Put.DOUBLE, "climber/climbOutput", 0.5);

  private final Loggable<String> climbState = BucketLog.loggable(Put.STRING, "climber/climbState");
  private final Loggable<String> hookToggleState = BucketLog.loggable(Put.STRING, "climber/hookState");
  private final Loggable<String> elevatorToggleState = BucketLog.loggable(Put.STRING, "climber/elevatorState");
  private final Loggable<Double> climberMotorVelocity = BucketLog.loggable(Put.DOUBLE, "climber/climberMotorVelocity");

  public ClimberSubsystem(Config config) {
    super(config);
  }

  @Override
  public void init() {
    climber1 = new WPI_TalonSRX(config.climberMotor_ID1);
    climber2 = new WPI_TalonSRX(config.climberMotor_ID2);
    if (config.enablePneumatics) {
      elevatorSolenoid =
        new DoubleSolenoid(PneumaticsModuleType.REVPH, config.elevatorSolenoid_ID1, config.elevatorSolenoid_ID2);
    }

    currentClimbPhase = ClimbPhase.Idle;
    currentClimbState = ClimbState.Idle;
  }

  @Override
  public void periodic() {}

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

  public void fixedHookToggler() { //uses R1 button
    return;
    // if (config.enablePneumatics) {
    //   fixedHookToggleState = !fixedHookToggleState;
    //   if (!fixedHookToggleState) {
    //     fixedHookSolenoid.set(Value.kForward);
    //     hookToggleState.log(LogLevel.GENERAL, "fixedHookTime");
    //   } else {
    //     fixedHookSolenoid.set(Value.kReverse);
    //     hookToggleState.log(LogLevel.GENERAL, "fixedHookTime");
    //   }
    // }
  }

  public void elevatorExtend() { //uses up button
    if (autoClimb) return;

    climber1.set(ControlMode.PercentOutput, climbOutput.currentValue());
    climber2.set(ControlMode.PercentOutput, climbOutput.currentValue());
    climberMotorVelocity.log(climber1.getSelectedSensorVelocity());

    climbState.log(LogLevel.GENERAL, "elevatorExtend");
  }

  public void elevatorRetract() { //uses down button
    if (autoClimb) return;

    climber1.set(ControlMode.PercentOutput, -climbOutput.currentValue());
    climber2.set(ControlMode.PercentOutput, -climbOutput.currentValue());
    climberMotorVelocity.log(climber1.getSelectedSensorVelocity());

    climbState.log(LogLevel.GENERAL, "elevatorRetract");
  }

  public void elevatorStop() {
    climber1.set(0);
    climber2.set(0);
    climbState.log(LogLevel.GENERAL, "climbStotopped");
  }

  public void climbAuto() { //uses TPAD button
    autoClimb = true;
    climbState.log(LogLevel.GENERAL, "climb2auto");

    // TODO: figure out behavior if it's stopped
    // my postulation is that we go to the next phase
    // i assume that controllers might stop it, finish the climb, then hit the button for the next phase

    if (currentClimbState == ClimbState.Finished) {
      if (currentClimbPhase == ClimbPhase.Idle) {
        autoClimbMid();
      } else if (currentClimbPhase == ClimbPhase.ClimbMid) {
        autoClimbHigh();
      } else if (currentClimbPhase == ClimbPhase.ClimbHigh) {
        autoClimbTraversal();
      }
    }
  }

  private void autoClimbMid() {
    currentClimbPhase = ClimbPhase.ClimbMid;

    // TODO: move these checks to periodic
    // get rid of all these functions
    // put it in periodic or smth after checking if the autoclimb toggle is true and what phase/state it's at
    // change phase and autoclimb state at the start and end, similar to what we have now?

    // once auto climb starts
    // toggle it
    // move motor forward
    // once hits limit switch, stop
    // toggles back
    // start retracting
    // stop when hit reverse limit switch
    // <end phase, human starts next phase>

    if (climber1.isFwdLimitSwitchClosed() == 1) climber1.set(0);
    if (climber2.isFwdLimitSwitchClosed() == 1) climber2.set(0);

    autoClimb = false;
  }

  private void autoClimbHigh() {
    currentClimbPhase = ClimbPhase.ClimbHigh;

    autoClimb = false;
  }

  private void autoClimbTraversal() {
    currentClimbPhase = ClimbPhase.ClimbTraversal;

    autoClimb = false;
  }

  // TODO: ADD A BUTTON FOR THIS
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
