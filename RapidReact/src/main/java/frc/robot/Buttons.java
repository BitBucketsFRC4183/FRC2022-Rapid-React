package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.utils.PS4Constants;

public class Buttons {

  //////////////////////////////////////////////////////////////////////////////
  // Driver
  Joystick driverControl = new Joystick(0);
  int swerveForward = PS4Constants.LEFT_STICK_Y.getId();
  int swerveStrafe = PS4Constants.LEFT_STICK_X.getId();
  int swerveRotation = PS4Constants.RIGHT_STICK_X.getId();
  JoystickButton zeroGyroscope = new JoystickButton(driverControl, PS4Constants.SHARE.getId());
  JoystickButton driverEnableClimber = new JoystickButton(driverControl, PS4Constants.PS4.getId());

  //////////////////////////////////////////////////////////////////////////////
  //Operator
  Joystick operatorControl = new Joystick(1);
  JoystickButton hubShoot = new JoystickButton(operatorControl, PS4Constants.R1.getId());
  JoystickButton lowShoot = new JoystickButton(operatorControl, PS4Constants.R2.getId());
  JoystickButton tarmacShootOrToggleElevator = new JoystickButton(operatorControl, PS4Constants.L1.getId());

  JoystickButton toggleIntake = new JoystickButton(operatorControl, PS4Constants.CIRCLE.getId());
  JoystickButton intake = new JoystickButton(operatorControl, PS4Constants.TRIANGLE.getId());
  JoystickButton outtake = new JoystickButton(operatorControl, PS4Constants.CROSS.getId());

  JoystickButton operatorEnableClimber = new JoystickButton(operatorControl, PS4Constants.PS4.getId());
  JoystickButton climbAuto = new JoystickButton(operatorControl, PS4Constants.TRACKPAD.getId());
  JoystickButton resetClimbStuff = new JoystickButton(operatorControl, PS4Constants.OPTIONS.getId());

  JoystickButton autoClimbStopLeft = new JoystickButton(operatorControl, PS4Constants.L2.getId());
  JoystickButton autoClimbStopRight = new JoystickButton(operatorControl, PS4Constants.R2.getId());

  POVButton elevatorRetract = new POVButton(operatorControl, 180); // DPAD down
  POVButton elevatorExtend = new POVButton(operatorControl, 0); // DPAD up
}
