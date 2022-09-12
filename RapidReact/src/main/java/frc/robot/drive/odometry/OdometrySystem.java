package frc.robot.drive.odometry;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.drive.DriveFactory;
import frc.robot.lib.System;
import frc.robot.lib.resources.SharedOut;

public class OdometrySystem implements System {

    final SharedOut<SwerveModuleState[]> moduleState;
    final AHRS ahrs;
    final SwerveDriveKinematics kinematics;
    final SwerveDriveOdometry odometry;
    final Field2d field;


    public OdometrySystem(SharedOut<SwerveModuleState[]> moduleState, AHRS ahrs, SwerveDriveKinematics kinematics, SwerveDriveOdometry odometry, Field2d field) {
        this.moduleState = moduleState;
        this.ahrs = ahrs;
        this.kinematics = kinematics;
        this.odometry = odometry;
        this.field = field;
    }

    @Override
    public void periodic(float delta, int iteration) {

        SwerveModuleState[] states = moduleState.sharedValue(); //call the function!
        odometry.update(ahrs.getRotation2d(), states);
        field.setRobotPose(odometry.getPoseMeters());

    }

    public Rotation2d rotationGyro() {
        return ahrs.getRotation2d();
    }


    public SwerveModuleState[] calculateFieldOrientedDesiredStates(double x, double y, double rot, double gain) {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(x,y,rot, ahrs.getRotation2d())
        );

        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveFactory.MAX_DRIVE_VELOCITY * gain);

        return states;
    }

}
