package frc.robot.subsystem.drive;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import frc.robot.lib.header.Header;
import frc.robot.lib.System;
import frc.robot.lib.header.MotorBuilder;
import frc.robot.lib.header.LogBuilder;

import static com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper.GearRatio.L2;
import static com.swervedrivespecialties.swervelib.SdsModuleConfigurations.MK4_L2;

public class DriveHeader implements Header {

    @Override
    public System init(LogBuilder logBuilder, MotorBuilder motorBuilder) {

        //constants
        double width = logBuilder.constantOnce(double.class, "track_width", 0.6096) / 2;
        double base = logBuilder.constantOnce(double.class, "wheel_base", 0.7112) / 2;
        double maxVel = 6380.0 / 60.0 * MK4_L2.getDriveReduction() * MK4_L2.getWheelDiameter() * Math.PI;
        double maxAngVel = maxVel / Math.hypot(width / 2, base) / 2;

        Translation2d[] wheelPositions = new Translation2d[] { new Translation2d(width, base), new Translation2d(width, -base), new Translation2d(-width, base), new Translation2d(-width, -base),};

        Gyro gyro = new AHRS(SPI.Port.kMXP, (byte)200);
        SwerveDriveKinematics kinematics = new SwerveDriveKinematics(wheelPositions);
        SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d(), new Pose2d()); //pose2d is starting pose
        SwerveModule[] modules = buildSwerveModuleStage(logBuilder, motorBuilder);

        return new DriveSystem(
                maxVel, modules,
                odometry,
                kinematics,
                gyro
        );

    }







    SwerveModule[] buildSwerveModuleStage(LogBuilder logBuilder, MotorBuilder motorBuilder) {
        //init ctors
        SwerveModule[] modules = new SwerveModule[4];

        ShuffleboardLayout frontLeft = logBuilder.tab()
                .getLayout("Front Left Module", BuiltInLayouts.kList)
                .withPosition(0, 0);
        ShuffleboardLayout frontRight = logBuilder.tab()
                .getLayout("Front Right Module", BuiltInLayouts.kList)
                .withPosition(2, 0);
        ShuffleboardLayout backLeft = logBuilder.tab()
                .getLayout("Back Left Module", BuiltInLayouts.kList)
                .withPosition(4, 0);
        ShuffleboardLayout backRight = logBuilder.tab()
                .getLayout("Back Right Module", BuiltInLayouts.kList)
                .withPosition(6, 0);

        modules[0] = Mk4SwerveModuleHelper.createFalcon500(frontLeft, L2, 1, 2, 9, -Math.toRadians(232.55));
        modules[1] = Mk4SwerveModuleHelper.createFalcon500(frontRight, L2, 7, 8, 12, -Math.toRadians(331.96 - 180));
        modules[2] = Mk4SwerveModuleHelper.createFalcon500(backLeft, L2, 5, 6, 11, -Math.toRadians(255.49));
        modules[3] = Mk4SwerveModuleHelper.createFalcon500(backRight, L2, 3, 4, 10, -Math.toRadians(70.66 + 180));

        return modules;
    }

}
