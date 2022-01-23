package frc.swervelib;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class SwerveConstants {
    public static double THETACONTROLLERkP = 1;
    public static double TRAJECTORYXkP = 1;
    public static double TRAJECTORYYkP = 1;
    public static TrapezoidProfile.Constraints THETACONTROLLERCONSTRAINTS;

    public static PIDController XPIDCONTROLLER = new PIDController(TRAJECTORYXkP, 0, 0);
    public static PIDController YPIDCONTROLLER = new PIDController(TRAJECTORYYkP, 0, 0);
}
