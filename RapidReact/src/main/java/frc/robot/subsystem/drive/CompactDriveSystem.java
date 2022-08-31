package frc.robot.subsystem.drive;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import frc.robot.lib.System;
import frc.robot.lib.log.Logger;
import frc.robot.lib.log.MotorBuilder;

import static com.swervedrivespecialties.swervelib.SdsModuleConfigurations.MK4_L2;
import static frc.robot.subsystem.drive.CompactDriveUtils.buildSwerveModuleStage;

public class CompactDriveSystem implements System {

    final double width;
    final double base;
    final double maxVel;
    final double maxAngularVel;

    final Gyro gyro;
    final SwerveDriveKinematics kinematics;
    final SwerveDriveOdometry odometry;
    final SwerveModule[] modules;

    final SimpleMotorFeedforward feedForward;

    public CompactDriveSystem(Logger logger, MotorBuilder motorBuilder) {

        //constants!
        width = logger.constantOnce(double.class, "track_width", 0.6096) / 2;
        base = logger.constantOnce(double.class, "wheel_base", 0.7112) / 2;

        maxVel = 6380.0 / 60.0 * MK4_L2.getDriveReduction() * MK4_L2.getWheelDiameter() * Math.PI;
        maxAngularVel = maxVel / Math.hypot(width / 2, base) / 2;

        feedForward = new SimpleMotorFeedforward(0.65292, 2.3053, 0.37626);

        //wheel
        Translation2d[] wheelPositions = new Translation2d[] { new Translation2d(width, base), new Translation2d(width, -base), new Translation2d(-width, base), new Translation2d(-width, -base)};

        gyro = new AHRS(SPI.Port.kMXP, (byte)200);
        kinematics = new SwerveDriveKinematics(wheelPositions);
        odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d(), new Pose2d()); //pose2d is starting pose
        modules = buildSwerveModuleStage(logger, motorBuilder);

    }

    @Override
    public void periodic(float delta) {

        SwerveModuleState[] states = new SwerveModuleState[4];

        for (int i = 0; i < modules.length; i++) {
            SwerveModule module = modules[i];

            states[i] = new SwerveModuleState(module.getDriveVelocity(), new Rotation2d(module.getSteerAngle()));
        }

        this.odometry.update(gyro.getRotation2d(), states);
    }

    //normal variables go here

}
