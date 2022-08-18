package frc.robot.subsystem.drive;

import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.lib.System;

public class D3Subsystem implements System {

    static final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(0.65292, 2.3053, 0.37626);
    final SwerveModule[] driveModules;

    public D3Subsystem(SwerveModule[] driveModules) {
        this.driveModules = driveModules;
    }

    public void command(SwerveModuleState[] states) {

        for(int i = 0; i < driveModules.length; i++)
        {
            double voltage = MathUtil.clamp(feedForward.calculate(states[i].speedMetersPerSecond), -12, 12);
            double radians = states[i].angle.getRadians();

            driveModules[i].set(voltage, radians);
        }
    }

}
