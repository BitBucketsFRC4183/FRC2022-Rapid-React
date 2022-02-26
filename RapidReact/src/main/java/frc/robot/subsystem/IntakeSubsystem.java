package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Robot;
import frc.robot.config.Config;
import frc.robot.log.*;
import frc.robot.simulator.CTREPhysicsSim;

public class IntakeSubsystem extends BitBucketsSubsystem {

  WPI_TalonSRX ballManagement;
  WPI_TalonSRX intake;
  //a boolean that checks whether the intake is running (true for on, false for off)
  public boolean toggleState;
  //double solenoid is used for the intake PCM
  DoubleSolenoid intakeSolenoid;

  //dashboard stuff
  private final Changeable<Double> percentOutput = BucketLog.changeable(Put.DOUBLE, "intake/percentOutput", 0.75);
  private final Changeable<Boolean> autoExtend = BucketLog.changeable(
    Put.BOOL,
    "intake/autoExtend",
    config.intake.defaultIntakeAutoExtend
  );
  private final Loggable<String> intakeState = BucketLog.loggable(Put.STRING, "intake/intakeState");

  public IntakeSubsystem(Config config) {
    super(config);
  }

  @Override
  public void init() {
    ballManagement = new WPI_TalonSRX(Config.ballManagementMotor_ID);
    intake = new WPI_TalonSRX(Config.intakeMotor_ID);
    if (config.enablePneumatics) {
      if (Robot.isSimulation()) {
        intakeSolenoid =
          new DoubleSolenoid(PneumaticsModuleType.CTREPCM, config.intakeSolenoid_ID1, config.intakeSolenoid_ID2);
      } else {
        intakeSolenoid =
          new DoubleSolenoid(PneumaticsModuleType.REVPH, config.intakeSolenoid_ID1, config.intakeSolenoid_ID2);
      }
    }
    if (Robot.isSimulation()) {
      // simulate the motors
      CTREPhysicsSim.getInstance().addTalonSRX(intake, .75,5100, false);
      CTREPhysicsSim.getInstance().addTalonSRX(ballManagement, .75,5100, false);
    }
  }

  @Override
  public void periodic() {}

  @Override
  public void disable() {
    intake.set(0);
  }

  //intaking, outtaking, and stop the intake
  public void spinForward() {
    if (autoExtend.currentValue() && config.enablePneumatics) {
      intakeSolenoid.set(Value.kForward);
    }
    intake.set(ControlMode.PercentOutput, percentOutput.currentValue());
    ballManagement.set(ControlMode.PercentOutput, percentOutput.currentValue());
    intakeState.log("intaking");
  }

  public void spinBackward() {
    if (autoExtend.currentValue() && config.enablePneumatics) {
      intakeSolenoid.set(Value.kForward);
    }
    intake.set(ControlMode.PercentOutput, -percentOutput.currentValue());
    ballManagement.set(ControlMode.PercentOutput, -percentOutput.currentValue());
    intakeState.log("outtaking");
  }

  public void stopSpin() {
    if (autoExtend.currentValue() && config.enablePneumatics) {
      intakeSolenoid.set(Value.kReverse);
    }
    intake.set(ControlMode.PercentOutput, 0);
    ballManagement.set(ControlMode.PercentOutput, 0);
    intakeState.log("stopped");
  }

  //toggles turning the intake on or off
  public void toggle() {
    if (config.enablePneumatics && autoExtend.currentValue() == false) {
      if (!toggleState) {
        intakeSolenoid.set(Value.kForward);
        intakeState.log("intaking");
        toggleState = true;
      } else {
        intakeSolenoid.set(Value.kReverse);
        intakeState.log("off");
        toggleState = false;
      }
    }
  }

  public void ballManagementForward() {
    ballManagement.set(ControlMode.PercentOutput, percentOutput.currentValue());
  }

  public void stopBallManagement() {
    ballManagement.set(ControlMode.PercentOutput, 0);
  }

}
