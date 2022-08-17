package frc.robot.subsystem.drive;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.RunCycle;

/**
 * The robot knows where it is...
 */
public class OdometrySubsystem extends SubsystemBase implements RunCycle {

    final Translation2d[] translations = new Translation2d[] {
            new Translation2d(TRACK_WIDTH_METERS_HALF, WHEEL_BASE_METERS_HALF), //frontLeft
            new Translation2d(TRACK_WIDTH_METERS_HALF, -WHEEL_BASE_METERS_HALF), //frontRight
            new Translation2d(-TRACK_WIDTH_METERS_HALF, WHEEL_BASE_METERS_HALF), //backLeft
            new Translation2d(-TRACK_WIDTH_METERS_HALF, -WHEEL_BASE_METERS_HALF), //backRight
    };

    final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(translations);


    @Override
    public void init() {

    }

    @Override
    public void periodic(float delta) {

    }

    @Override
    public void stop() {

    }

    public Pose2d absolute() {
        return null; //TODO
    }
}
