package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.lib.exec.Pressable;
import frc.robot.utils.PS4;

public class Buttons {







  //////////////////////////////////////////////////////////////////////////////
  // Driver
  public Joystick driverControl = new Joystick(0);
  public int swerveForward = PS4.LEFT_STICK_Y;
  public int swerveStrafe = PS4.LEFT_STICK_X;
  public int swerveRotation = PS4.RIGHT_STICK_X;
  public JoystickButton driverEnableClimber = new JoystickButton(driverControl, PS4.PS4);
  public JoystickButton resetOdometry = new JoystickButton(driverControl, PS4.TRACKPAD);
  public JoystickButton slowDrive = new JoystickButton(driverControl, PS4.R2);

  public Pressable SLOW_DRIVE = Pressable.from(new JoystickButton(driverControl, PS4.R2));
  public Pressable AIM_DRIVE = Pressable.from(new JoystickButton(driverControl, PS4.CIRCLE));

  //////////////////////////////////////////////////////////////////////////////
  //Operator
  public Joystick operatorControl = new Joystick(1);
  public JoystickButton hubSpinUp = new JoystickButton(operatorControl, PS4.R1);
  public JoystickButton lowShoot = new JoystickButton(operatorControl, PS4.R2);
  public JoystickButton toggleElevator = new JoystickButton(operatorControl, PS4.L1);
  public JoystickButton feedInFire = new JoystickButton(operatorControl, PS4.L2);
  public JoystickButton autoShoot = new JoystickButton(operatorControl, PS4.SQUARE);
  public JoystickButton autoShootOne = new JoystickButton(operatorControl, PS4.R_STICK);



  public JoystickButton toggleIntake = new JoystickButton(operatorControl, PS4.CIRCLE);
  public JoystickButton intake = new JoystickButton(operatorControl, PS4.TRIANGLE);
  public JoystickButton outtake = new JoystickButton(operatorControl, PS4.CROSS);

  public JoystickButton operatorEnableClimber = new JoystickButton(operatorControl, PS4.PS4);
  public JoystickButton climbAuto = new JoystickButton(operatorControl, PS4.TRACKPAD);
  public JoystickButton resetClimbStuff = new JoystickButton(operatorControl, PS4.OPTIONS);

  public JoystickButton autoClimbStopLeft = new JoystickButton(operatorControl, PS4.L2);
  public JoystickButton autoClimbStopRight = new JoystickButton(operatorControl, PS4.R2);

  public JoystickButton rgb = new JoystickButton(operatorControl, PS4.SHARE);

  public POVButton elevatorRetract = new POVButton(operatorControl, 180); // DPAD down
  public POVButton elevatorExtend = new POVButton(operatorControl, 0); // DPAD up
}
