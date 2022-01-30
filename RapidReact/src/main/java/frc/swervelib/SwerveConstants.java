package frc.swervelib;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class SwerveConstants {
    public static double THETACONTROLLERkP = 1;
    public static double TRAJECTORYXkP = 1;
    public static double TRAJECTORYYkP = 1;
    public static TrapezoidProfile.Constraints THETACONTROLLERCONSTRAINTS = new TrapezoidProfile.Constraints(500, 100);

    public static PIDController XPIDCONTROLLER = new PIDController(.0001, 0.0001, 0.0001);
    public static PIDController YPIDCONTROLLER = XPIDCONTROLLER;
}
