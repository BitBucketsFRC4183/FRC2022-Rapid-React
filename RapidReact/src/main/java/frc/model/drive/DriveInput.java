package frc.model.drive;

import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.drive.Drive;
import frc.robot.lib.data.Const;
import frc.robot.lib.data.Container;

public interface DriveInput {

    Container INPUT = Drive.DRIVE.sub("input");

    double X_LIMIT = INPUT.constant(Const.NUMBER, "x_limiter", 2.0).get();
    double Y_LIMIT = INPUT.constant(Const.NUMBER, "y_limiter", 2.0).get();

    SlewRateLimiter X_LIMITER = new SlewRateLimiter(X_LIMIT);
    SlewRateLimiter Y_LIMITER = new SlewRateLimiter(Y_LIMIT);

    boolean shouldSlowDrive();
    boolean shouldAimDrive();

    double normalizedX();
    double normalizedY();
    double rotation();

}
