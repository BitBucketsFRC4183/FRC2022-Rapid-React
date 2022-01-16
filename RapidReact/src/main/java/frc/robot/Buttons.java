package frc.robot;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.utils.PS4Constants;

public class Buttons {

    Joystick driverControl = new Joystick(0);
    int SwerveForward = PS4Constants.LEFT_STICK_Y.getId();
    int SwerveStrafe = PS4Constants.LEFT_STICK_X.getId();
    int SwerveRotation = PS4Constants.RIGHT_STICK_X.getId();
    JoystickButton zeroGyroscope = new JoystickButton(driverControl, PS4Constants.SHARE.getId());
}
