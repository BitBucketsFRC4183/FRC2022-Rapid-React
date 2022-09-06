package frc.robot.drive;

import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.lib.System;
import frc.robot.lib.resources.SharedFunction;

public class DriveSystem implements System {

    final SwerveModule[] swerve;

    public DriveSystem(SwerveModule[] swerve) {
        this.swerve = swerve;
    }

    @SharedFunction
    public SwerveModuleState[] currentStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];

        for (int i = 0; i < swerve.length; i++) {
            SwerveModule module = swerve[i];

            states[i] = new SwerveModuleState(module.getDriveVelocity(), new Rotation2d(module.getSteerAngle()));
        }

        return states;
    }

    public void driveAt(SwerveModuleState[] states) {

        for(int i = 0; i < swerve.length; i++)
        {
            double voltage = MathUtil.clamp(Drive.FEEDFORWARD.calculate(states[i].speedMetersPerSecond), -12, 12);
            double radians = states[i].angle.getRadians();

            swerve[i].set(voltage, radians);
        }

    }



}
