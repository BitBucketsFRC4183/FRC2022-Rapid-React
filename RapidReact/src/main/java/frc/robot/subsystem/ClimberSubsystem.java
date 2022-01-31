package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
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

  public ClimberSubsystem(Config config) {
    super(config);
  }

  @Override
  public void init() {
    climber = new WPI_TalonSRX(Config.climberMotor_ID);
    elevatorSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 2, 3);
    fixedHookSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 4, 5);
  }

  @Override
  public void periodic() {}

  @Override
  public void disable() {}

  public void enableClimber() { // uses 2 PS button
    enabledClimber = true;
    logger().logString(LogLevel.GENERAL, "climb_state", "climberEnabled");
  }

  public void disableClimber() {
    enabledClimber = false;
    logger().logString(LogLevel.GENERAL, "climb_state", "climberDisabled");
  }

  public void fixedHookToggler() { //uses R1 button
    fixedHookToggleState = !fixedHookToggleState;
    if (fixedHookToggleState == false){
      fixedHookSolenoid.set(Value.kForward);
      logger().logString(LogLevel.GENERAL, "fixedHookToggle_state", "fixedHookTime");
    } else{
      fixedHookSolenoid.set(Value.kReverse);
      logger().logString(LogLevel.GENERAL, "fixedHookToggle_state", "fixedHookTime");
    }
  }

  public void elevatorExtend() { //uses up button
    logger().logString(LogLevel.GENERAL, "climb_state", "elevatorExtend");
  }

  public void elevatorRetract() { //uses down button
    logger().logString(LogLevel.GENERAL, "climb_state", "elevatorRetract");
  }

  public void climbAuto() { //uses TPAD button
    autoClimb = true;
    logger().logString(LogLevel.GENERAL, "climb_state", "time2autoclimb");
  }

  public void elevatorToggle() {
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
