package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.config.Config;
import frc.robot.log.LogLevel;

public class IntakeSubsystem extends BitBucketsSubsystem {

  private WPI_TalonSRX intake;
  
  public IntakeSubsystem(Config config) {
    super(config);
  }

  @Override
  public void init() {
    intake = new WPI_TalonSRX(Config.INTAKE_MOTOR_ID);
  }

  @Override
  public void periodic() {
  }

  @Override
  public void disable() {}

  public void spinForward() {
    intake.set(ControlMode.PercentOutput, 0.75);
    logger().logString(LogLevel.GENERAL, "intakeState", "Intaking");
  }


  public void spinBackward() {
    intake.set(ControlMode.PercentOutput, -0.75);
    logger().logString(LogLevel.GENERAL, "intakeState", "Outtaking");
  }

  public void stopSpin() {
    intake.set(ControlMode.PercentOutput, 0);
    logger().logString(LogLevel.GENERAL, "intakeState", "Stopped");
  }

  // public void toggle() {
  //   if (toggleState == false) {
  //       intakeSolenoid.set(Value.kForward);
  //       toggleState = true;
  //   }
    
  //   else {
  //       intakeSolenoid.set(Value.kReverse);
  //       toggleState = false; 
  //   }
  // }
}
