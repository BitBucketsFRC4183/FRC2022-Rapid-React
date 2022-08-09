package frc.robot.subsystem.drive;

import static com.swervedrivespecialties.swervelib.SdsModuleConfigurations.MK4_L2;

public interface DriveConstants {

    double trackWidthMeters = 0.6096;
    double TRACK_WIDTH_METERS_HALF = trackWidthMeters / 2.0;
    double wheelBaseMeters = 0.7112;
    double WHEEL_BASE_METERS_HALF = wheelBaseMeters / 2.0;

    double MAX_VEL_METERS_PER_SECOND = 6380.0 / 60.0 * MK4_L2.getDriveReduction() * MK4_L2.getWheelDiameter() * Math.PI;
    double maxAngVelRadsPerSecond = MAX_VEL_METERS_PER_SECOND / Math.hypot(TRACK_WIDTH_METERS_HALF, WHEEL_BASE_METERS_HALF);

}
