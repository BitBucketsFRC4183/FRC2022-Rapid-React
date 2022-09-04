package frc.robot.drive;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.lib.header.SystemHeader;
import frc.robot.lib.resources.SharedOut;

import static frc.robot.drive.Drive.WHEEL_BASE;
import static frc.robot.drive.Drive.WIDTH;

public interface Odometry {

    //Container ODOMETRY = Container.GLOBAL.sub("odometry");

    SystemHeader HEADER = (context) -> {
        Translation2d[] wheelPositions = new Translation2d[] {
                new Translation2d(WIDTH, WHEEL_BASE),
                new Translation2d(WIDTH, -WHEEL_BASE),
                new Translation2d(-WIDTH, WHEEL_BASE),
                new Translation2d(-WIDTH, -WHEEL_BASE)
        };

        AHRS gyro = new AHRS(SPI.Port.kMXP, (byte)200);
        SwerveDriveKinematics kinematics = new SwerveDriveKinematics(wheelPositions);
        SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d(), new Pose2d());
        Field2d field2d = new Field2d();

        SharedOut<SwerveModuleState[]> motorState = context.shareOutput(Drive.SHARED_SWERVE);

        return new OdometrySystem(motorState, gyro, kinematics, odometry);
    };

}
