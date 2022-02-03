package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.config.Config;
import frc.robot.log.LogLevel;

public class ClimberSubsystem extends BitBucketsSubsystem {

  private WPI_TalonSRX climber;
  private boolean enabledClimber;
  private boolean fixedHookToggleState;
  private boolean elevatorToggle;
  private boolean autoClimb;
  DoubleSolenoid elevatorSolenoid;
  DoubleSolenoid fixedHookSolenoid;
  private double climbOutput = .5;

  public ClimberSubsystem(Config config) {
    super(config);
  }

  @Override
  public void init() {
    climber = new WPI_TalonSRX(Config.climberMotor_ID);
    if (config.enablePneumatics) {
      elevatorSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 2, 3);
      fixedHookSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 4, 5);
    }

    logger()
      .subscribeNum(
        "climbSpeed",
        e -> {
          climbOutput = e.doubleValue();
        },
        climbOutput
      );
  }

  @Override
  public void periodic() {}

  @Override
  public void disable() {
    climber.set(0);
  }

  public void enableClimber() { // uses 2 PS button
    enabledClimber = true;
    logger().logString(LogLevel.GENERAL, "climb_state", "climberEnabled");
  }

  public void disableClimber() {
    enabledClimber = false;
    logger().logString(LogLevel.GENERAL, "climb_state", "climberDisabled");
  }

  public void fixedHookToggler() { //uses R1 button
    if (config.enablePneumatics) {
      fixedHookToggleState = !fixedHookToggleState;
      if (fixedHookToggleState == false) {
        fixedHookSolenoid.set(Value.kForward);
        logger().logString(LogLevel.GENERAL, "fixedHookToggle_state", "fixedHookTime");
      } else {
        fixedHookSolenoid.set(Value.kReverse);
        logger().logString(LogLevel.GENERAL, "fixedHookToggle_state", "fixedHookTime");
      }
    }
  }

  public void elevatorExtend() { //uses up button
    climber.set(ControlMode.PercentOutput, climbOutput);
    logger().logString(LogLevel.GENERAL, "climb_state", "elevatorExtend");
  }

  public void elevatorRetract() { //uses down button
    climber.set(ControlMode.PercentOutput, -climbOutput);
    logger().logString(LogLevel.GENERAL, "climb_state", "elevatorRetract");
  }

  public void elevatorStop() {
    climber.set(0);
    logger().logString(LogLevel.GENERAL, "climb_state", "climbStopped");
  }

  public void climbAuto() { //uses TPAD button
    autoClimb = true;
    logger().logString(LogLevel.GENERAL, "climb_state", "time2autoclimb");
  }

  public void elevatorToggle() {
    if (config.enablePneumatics) {
      elevatorToggle = !elevatorToggle;
      if (elevatorToggle == false) {
        elevatorSolenoid.set(Value.kForward);
        logger().logString(LogLevel.GENERAL, "elevatorToggle_state", "ClimbTime");
      } else {
        elevatorSolenoid.set(Value.kReverse);
        logger().logString(LogLevel.GENERAL, "elevatorToggle_state", "NotClimbTime");
      }
    }
  }
}
