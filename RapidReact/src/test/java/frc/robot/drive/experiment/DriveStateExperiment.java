package frc.robot.drive.experiment;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import org.junit.Test;

import static frc.robot.subsystem.drive.DriveHeader.TRACK_WIDTH_METERS_HALF;
import static frc.robot.subsystem.drive.DriveHeader.WHEEL_BASE_METERS_HALF;

public class DriveStateExperiment {

    private final Translation2d[] translations = new Translation2d[] {
            new Translation2d(TRACK_WIDTH_METERS_HALF, WHEEL_BASE_METERS_HALF), //frontLeft
            new Translation2d(TRACK_WIDTH_METERS_HALF, -WHEEL_BASE_METERS_HALF), //frontRight
            new Translation2d(-TRACK_WIDTH_METERS_HALF, WHEEL_BASE_METERS_HALF), //backLeft
            new Translation2d(-TRACK_WIDTH_METERS_HALF, -WHEEL_BASE_METERS_HALF), //backRight
    };

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(translations);


    @Test
    public void rotationsAddedTogether() {
        System.out.println(radsThirty.plus(radsSixty));
    }

    @Test
    public void test() {
        ChassisSpeeds speed1 = new ChassisSpeeds(0,0,0);

        kinematics.toSwerveModuleStates(speed1);


    }

}
