package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.config.Config;
import frc.robot.log.LogLevel;

public class IntakeSubsystem extends BitBucketsSubsystem {

  private WPI_TalonSRX intake;
  //a boolean that checks whether the intake is running (true for on, false for off)
  public boolean toggleState;
  //double solenoid is used for the intake PCM
  DoubleSolenoid intakeSolenoid;

  //sends value of motor speed to Smart Dashboard
 double percentOutput = 0.75;
  
  public IntakeSubsystem(Config config) {
    super(config);
  }

  @Override
  public void init() {
    intake = new WPI_TalonSRX(Config.intakeMotor_ID);
    intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);

    //shows the speed of intake on the smart dashboard
    logger().subscribeNum("outputSpeed",(e) -> {
        percentOutput = e.doubleValue();
    }, 0.75);
    
  }

  @Override
  public void periodic() {
  }

  @Override
  public void disable() {
    intake.set(0);
  }

  //intaking, outtaking, and stop the intake
  public void spinForward() {
    intake.set(ControlMode.PercentOutput, percentOutput);
    logger().logString(LogLevel.GENERAL, "intakeState", "Intaking");
  }

  public void spinBackward() {
    intake.set(ControlMode.PercentOutput, percentOutput);
    logger().logString(LogLevel.GENERAL, "intakeState", "Outtaking");
  }

  public void stopSpin() {
    intake.set(ControlMode.PercentOutput, 0);
    logger().logString(LogLevel.GENERAL, "intakeState", "Stopped");
  }

  //toggles turning the intake on or off
  public void toggle() {
    if (toggleState == false) {
        intakeSolenoid.set(Value.kForward);
        logger().logString(LogLevel.GENERAL, "intakeState", "Intaking");
        toggleState = true;
    }
    
    else {
        intakeSolenoid.set(Value.kReverse);
        logger().logString(LogLevel.GENERAL, "intakeState", "Off");
        toggleState = false; 
    }
  }
}
