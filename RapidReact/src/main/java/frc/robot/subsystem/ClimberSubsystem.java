package frc.robot.subsystem;

import frc.robot.config.Config;
import frc.robot.log.LogLevel;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class ClimberSubsystem extends BitBucketsSubsystem {
  
    private WPI_TalonSRX climber;
    private boolean enabledClimber;
    private boolean disabledClimber;

    public ClimberSubsystem(Config config) {
      super(config);
    }

    @Override
    public void init() {
      climber = new WPI_TalonSRX(Config.CLIMBER_MOTOR_ID);
    }
  
    @Override
    public void periodic() {

    }
  
    @Override
    public void disable() {

    }

    public void enableClimber(){ // uses 2 PS button
      enabledClimber = true;
      logger().logString(LogLevel.GENERAL, "climb_state", "climberEnabled");
    }

    public void disableClimber(){
      disabledClimber = true;
    }

    public void fixedHookToggler(){ //uses R1 button
      
    }

    public void elevatorToggle(){ //uses 4 button

    }

    public void elevatorExtend(){ //uses up button

    }

    public void elevatorRetract(){ //uses down button

    }

    public void climbAuto(){ //uses TPAD button

  }
}  
