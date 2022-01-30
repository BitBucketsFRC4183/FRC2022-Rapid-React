package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.config.Config;
import frc.robot.log.LogLevel;

public class ClimberSubsystem extends BitBucketsSubsystem {

  private WPI_TalonSRX climber;
  private boolean enabledClimber;
  private boolean disabledClimber;
  private boolean fixedHook;
  private boolean elevatorToggle;
  private boolean elevatorExtend;
  private boolean elevatorRetract;
  private boolean autoClimb;
  private boolean toggleState;
  DoubleSolenoid climberSolenoid;

  public ClimberSubsystem(Config config) {
    super(config);
  }

  @Override
  public void init() {
    climber = new WPI_TalonSRX(Config.CLIMBER_MOTOR_ID);
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
    disabledClimber = true;
    logger().logString(LogLevel.GENERAL, "climb_state", "climberDisabled");
  }

  public void fixedHookToggler() { //uses R1 button
    fixedHook = true;
    logger().logString(LogLevel.GENERAL, "climb_state", "fixedHookEnabled");
  }

  public void elevatorToggle() { //uses 4 button
    elevatorToggle = true;
    logger().logString(LogLevel.GENERAL, "climb_state", "elevatorEnabled");
  }

  public void elevatorExtend() { //uses up button
    elevatorExtend = true;
    logger().logString(LogLevel.GENERAL, "climb_state", "elevatorExtend");
  }

  public void elevatorRetract() { //uses down button
    elevatorRetract = true;
    logger().logString(LogLevel.GENERAL, "climb_state", "elevatorRetract");
  }

  public void climbAuto() { //uses TPAD button
    autoClimb = true;
    logger().logString(LogLevel.GENERAL, "climb_state", "time2autoclimb");
  }

  public void toggle() {
    if (toggleState == false) {
      climberSolenoid.set(Value.kForward);
      logger().logString(LogLevel.GENERAL, "climb_state", "ClimbTime");
      toggleState = true;
    } else {
      climberSolenoid.set(Value.kReverse);
      logger().logString(LogLevel.GENERAL, "climb_state", "NotClimbTime");
      toggleState = false;
    }
  }
}
