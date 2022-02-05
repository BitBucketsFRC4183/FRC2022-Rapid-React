package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.config.Config;
import frc.robot.log.*;

public class ClimberSubsystem extends BitBucketsSubsystem {

  private WPI_TalonSRX climber;
  private boolean enabledClimber;
  private boolean fixedHookToggleState;
  private boolean elevatorToggle;
  private boolean autoClimb;
  DoubleSolenoid elevatorSolenoid;
  DoubleSolenoid fixedHookSolenoid;

  private final Changeable<Double> climbOutput = BucketLog.changeable(Put.DOUBLE, "climber/climbOutput", 0.5);

  private final Loggable<String> climbState = BucketLog.loggable(Put.STRING, "climber/climbState");
  private final Loggable<String> hookToggleState = BucketLog.loggable(Put.STRING, "climber/hookState");
  private final Loggable<String> elevatorToggleState = BucketLog.loggable(Put.STRING, "climber/elevatorState");

  public ClimberSubsystem(Config config) {
    super(config);
  }

  @Override
  public void init() {
    climber = new WPI_TalonSRX(config.climberMotor_ID);
    if (config.enablePneumatics) {
      elevatorSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, config.elevatorSolenoid_ID1, config.elevatorSolenoid_ID2);
      fixedHookSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, config.fixedHookSolenoid_ID1, config.fixedHookSolenoid_ID2);
    }
  }

  @Override
  public void periodic() {}

  @Override
  public void disable() {
    climber.set(0);
  }

  public void enableClimber() { // uses 2 PS button
    enabledClimber = true;
    climbState.log(LogLevel.GENERAL, "climberEnabled");
  }

  public void disableClimber() {
    enabledClimber = false;
    climbState.log(LogLevel.GENERAL, "climberDisabled");
  }

  public void fixedHookToggler() { //uses R1 button
    if (config.enablePneumatics) {
      fixedHookToggleState = !fixedHookToggleState;
      if (!fixedHookToggleState) {
        fixedHookSolenoid.set(Value.kForward);
        hookToggleState.log(LogLevel.GENERAL, "fixedHookTime");
      } else {
        fixedHookSolenoid.set(Value.kReverse);
        hookToggleState.log(LogLevel.GENERAL, "fixedHookTime");
      }
    }
  }

  public void elevatorExtend() { //uses up button
    climber.set(ControlMode.PercentOutput, climbOutput.currentValue());

    climbState.log(LogLevel.GENERAL, "elevatorExtend");
  }

  public void elevatorRetract() { //uses down button
    climber.set(ControlMode.PercentOutput, -climbOutput.currentValue());

    climbState.log(LogLevel.GENERAL, "elevatorRetract");
  }

  public void elevatorStop() {
    climber.set(0);
    climbState.log(LogLevel.GENERAL, "climbStopped");
  }

  public void climbAuto() { //uses TPAD button
    autoClimb = true;
    climbState.log(LogLevel.GENERAL, "climb2auto");
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

  public boolean getEnabledClimber(){
    return enabledClimber;
  }
}
