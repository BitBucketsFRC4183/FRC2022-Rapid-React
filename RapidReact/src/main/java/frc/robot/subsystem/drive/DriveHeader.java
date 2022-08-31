package frc.robot.subsystem.drive;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import frc.robot.lib.Header;
import frc.robot.lib.System;
import frc.robot.lib.log.MotorBuilder;
import frc.robot.lib.log.Logger;

import static com.swervedrivespecialties.swervelib.SdsModuleConfigurations.MK4_L2;
import static frc.robot.subsystem.drive.CompactDriveUtils.buildSwerveModuleStage;

public class DriveHeader implements Header {



    @Override
    public System init(Logger logger, MotorBuilder motorBuilder) {

        //constants
        double width = logger.constantOnce(double.class, "a", 0.6096) / 2;
        double base = logger.constantOnce(double.class, "BASE_KEY", 0.7112) / 2;
        double maxVel = 6380.0 / 60.0 * MK4_L2.getDriveReduction() * MK4_L2.getWheelDiameter() * Math.PI;
        double maxAngVel = maxVel / Math.hypot(width / 2, base) / 2;

        Translation2d[] wheelPositions = new Translation2d[] { new Translation2d(width, base), new Translation2d(width, -base), new Translation2d(-width, base), new Translation2d(-width, -base),};

        Gyro gyro = new AHRS(SPI.Port.kMXP, (byte)200);
        SwerveDriveKinematics kinematics = new SwerveDriveKinematics(wheelPositions);
        SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d(), new Pose2d()); //pose2d is starting pose
        SwerveModule[] modules = buildSwerveModuleStage(logger, motorBuilder);

        return new DriveSystem(
                maxVel, modules,
                odometry,
                kinematics,
                gyro
        );

    }









}
