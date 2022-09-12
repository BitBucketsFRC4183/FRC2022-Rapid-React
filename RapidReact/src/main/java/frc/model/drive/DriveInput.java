package frc.model.drive;

import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.drive.DriveFactory;
import frc.robot.lib.data.Const;
import frc.robot.lib.data.Container;

public interface DriveInput {

    boolean shouldSlowDrive();
    boolean shouldAimDrive();

    double normalizedX();
    double normalizedY();
    double rotation();

}
