package frc.robot.drive;

import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import frc.robot.lib.System;
import frc.robot.lib.data.Const;
import frc.robot.lib.data.Container;
import frc.robot.lib.header.SystemHeader;

import static com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper.GearRatio.L2;
import static com.swervedrivespecialties.swervelib.SdsModuleConfigurations.MK4_L2;

public interface Drive {

    //          CONTAINER
    Container DRIVE = Container.GLOBAL.sub("drive");


    //          CONST
    String SHARED_SWERVE = "shared_swerve";

    double WIDTH = DRIVE.constant(Const.NUMBER, "track_width", 0.6096).get() / 2;
    double WHEEL_BASE = DRIVE.constant(Const.NUMBER, "wheel_base", 0.7112).get() / 2;

    double VISION_DRIVE_THRESHOLD_DEG = 1.0;
    double MAX_DRIVE_VELOCITY = 6380.0 / 60.0 * MK4_L2.getDriveReduction() * MK4_L2.getWheelDiameter() * Math.PI;
    double MAX_ANGULAR_VEL = Math.hypot(WIDTH / 2, WHEEL_BASE) / 2; //TODO math check

    SimpleMotorFeedforward FEEDFORWARD = new SimpleMotorFeedforward(0.65292, 2.3053, 0.37626);

    SystemHeader HEADER = (context) -> {
        SwerveModule[] modules = new SwerveModule[4];

        ShuffleboardLayout frontLeft = context.tab()
                .getLayout("Front Left Module", BuiltInLayouts.kList)
                .withPosition(0, 0);
        ShuffleboardLayout frontRight = context.tab()
                .getLayout("Front Right Module", BuiltInLayouts.kList)
                .withPosition(2, 0);
        ShuffleboardLayout backLeft = context.tab()
                .getLayout("Back Left Module", BuiltInLayouts.kList)
                .withPosition(4, 0);
        ShuffleboardLayout backRight = context.tab()
                .getLayout("Back Right Module", BuiltInLayouts.kList)
                .withPosition(6, 0);

        modules[0] = Mk4SwerveModuleHelper.createFalcon500(frontLeft, L2, 1, 2, 9, -Math.toRadians(232.55));
        modules[1] = Mk4SwerveModuleHelper.createFalcon500(frontRight, L2, 7, 8, 12, -Math.toRadians(331.96 - 180));
        modules[2] = Mk4SwerveModuleHelper.createFalcon500(backLeft, L2, 5, 6, 11, -Math.toRadians(255.49));
        modules[3] = Mk4SwerveModuleHelper.createFalcon500(backRight, L2, 3, 4, 10, -Math.toRadians(70.66 + 180));

        DriveSystem system = new DriveSystem(modules);

        context.shareInput(SHARED_SWERVE, system::currentStates);

        return system;
    };





}
