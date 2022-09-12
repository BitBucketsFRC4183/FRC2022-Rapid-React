package frc.log;

import frc.robot.lib.data.Const;
import frc.robot.lib.data.Container;

import static com.swervedrivespecialties.swervelib.SdsModuleConfigurations.MK4_L2;

public interface DriveConst {

    Container CONTAINER = Container.GLOBAL.sub("drive");

    String SHARED_SWERVE = "shared_swerve";

    double WIDTH = CONTAINER.constant(Const.NUMBER, "track_width_half", 0.3048).get();
    double WHEEL_BASE = CONTAINER.constant(Const.NUMBER, "wheel_base_half", 0.3556).get();

    double VISION_DRIVE_THRESHOLD_DEG = 1.0;
    double MAX_DRIVE_VELOCITY = 6380.0 / 60.0 * MK4_L2.getDriveReduction() * MK4_L2.getWheelDiameter() * Math.PI;
    double MAX_ANGULAR_VEL = Math.hypot(WIDTH / 2, WHEEL_BASE) / 2;

}
