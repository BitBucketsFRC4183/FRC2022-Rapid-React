package frc.robot.subsystem.drive;

import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import frc.robot.lib.System;

public class DriveSystem implements System {

    final double maxVel;
    final SwerveModule[] driveModules;
    final SwerveDriveOdometry odometry;
    final SwerveDriveKinematics kinematics;
    final Gyro gyro;

    static final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(0.65292, 2.3053, 0.37626);

    public DriveSystem(double maxVel, SwerveModule[] driveModules, SwerveDriveOdometry odometry, SwerveDriveKinematics kinematics, Gyro gyro) {
        this.maxVel = maxVel;
        this.driveModules = driveModules;
        this.odometry = odometry;
        this.kinematics = kinematics;
        this.gyro = gyro;
    }

    public void driveFieldOriented(double xMeterSecond, double yMeterSecond, double omegaRadSecond) {
        driveAt(
                kinematics.toSwerveModuleStates(
                        ChassisSpeeds.fromFieldRelativeSpeeds(xMeterSecond, yMeterSecond, omegaRadSecond, gyro.getRotation2d())
                )
        );
    }

    public void driveAt(SwerveModuleState[] states) {

        double max = maxVel * (slow ? 1 : 0.75);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, max);

        for(int i = 0; i < driveModules.length; i++)
        {
            double voltage = MathUtil.clamp(feedForward.calculate(states[i].speedMetersPerSecond), -12, 12);
            double radians = states[i].angle.getRadians();

            driveModules[i].set(voltage, radians);
        }

    }

    boolean slow;

    public void slowMode() {
        if (slow) {
            slow = false;
            return;
        }

        slow = true;
    }


}
