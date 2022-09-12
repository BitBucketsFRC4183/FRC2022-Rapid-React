package frc.log;

import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.drive.DriveFactory;
import frc.robot.lib.data.Const;
import frc.robot.lib.data.Container;

public interface DriveInputConst {

    Container CONTAINER = DriveConst.CONTAINER.sub("input");

    double X_LIMIT = CONTAINER.constant(Const.NUMBER, "x_limiter", 2.0).get();
    double Y_LIMIT = CONTAINER.constant(Const.NUMBER, "y_limiter", 2.0).get();

    SlewRateLimiter X_LIMITER = new SlewRateLimiter(X_LIMIT);
    SlewRateLimiter Y_LIMITER = new SlewRateLimiter(Y_LIMIT);

}
