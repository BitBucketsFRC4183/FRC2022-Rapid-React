package frc.robot.subsystem.odo;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import frc.robot.lib.System;
import frc.robot.lib.header.ConstantBuilder;
import frc.robot.lib.header.Header;
import frc.robot.lib.header.MotorBuilder;

public class OdometryHeader implements Header {

    @Override
    public System init(ConstantBuilder constantBuilder, MotorBuilder motorBuilder) {

        //constants for kinematics
        double width = constantBuilder.constantOnce(double.class, "track_width", 0.6096) / 2;
        double base = constantBuilder.constantOnce(double.class, "wheel_base", 0.7112) / 2;

        var t = new Translation2d[] {
                new Translation2d(width, base), //frontLeft
                new Translation2d(width, -base), //frontRight
                new Translation2d(-width, base), //backLeft
                new Translation2d(-width, -base), //backRight
        };



        return new O2Subsystem(
                new AHRS(SPI.Port.kMXP, (byte)200),
                new SwerveDriveOdometry(),
                new SwerveDriveKinematics()

        );
    }
}
