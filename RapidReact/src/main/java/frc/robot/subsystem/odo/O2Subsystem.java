package frc.robot.subsystem.odo;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import frc.robot.lib.System;

public class O2Subsystem implements System {

    final Gyro gyro;
    final SwerveDriveOdometry odometry;
    final SwerveDriveKinematics kinematics;

    final Translation2d[] translations;

    public O2Subsystem(Gyro gyro, SwerveDriveOdometry odometry, SwerveDriveKinematics kinematics, Translation2d[] translations) {
        this.gyro = gyro;
        this.odometry = odometry;
        this.kinematics = kinematics;
        this.translations = translations;
    }

    @Override
    public void periodic(float delta) {
        kinematics.
    }
}
