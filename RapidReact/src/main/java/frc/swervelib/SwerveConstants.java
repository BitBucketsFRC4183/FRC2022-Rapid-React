package frc.swervelib;

import edu.wpi.first.math.controller.PIDController;

public class SwerveConstants {
    public static PIDController XPIDCONTROLLER = new PIDController(1, 0, 0);
    public static PIDController YPIDCONTROLLER = XPIDCONTROLLER;
}
