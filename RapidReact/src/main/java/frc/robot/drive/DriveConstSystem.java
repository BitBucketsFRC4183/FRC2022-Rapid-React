package frc.robot.drive;

import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.log.DriveConst;
import frc.robot.lib.System;

public class DriveConstSystem implements System, DriveConst.Modify {

    final SwerveModule[] swerve;

    public DriveConstSystem(SwerveModule[] swerve) {
        this.swerve = swerve;

    }

    @Override
    public SwerveModuleState[] currentStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];

        for (int i = 0; i < swerve.length; i++) {
            SwerveModule module = swerve[i];

            states[i] = new SwerveModuleState(module.getDriveVelocity(), new Rotation2d(module.getSteerAngle()));
        }

        return states;
    }

    @Override
    public void driveAt(SwerveModuleState[] states) {

        for(int i = 0; i < swerve.length; i++)
        {
            double voltage = MathUtil.clamp(DriveFactory.FEEDFORWARD.calculate(states[i].speedMetersPerSecond), -12, 12);
            double radians = states[i].angle.getRadians();

            swerve[i].set(voltage, radians);
        }

    }



}
