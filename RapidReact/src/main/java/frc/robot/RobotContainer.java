package frc.robot;

import frc.robot.drive.command.DriveDirectorTeleop;
import frc.robot.lib.RobotInit;

public class RobotContainer implements RobotInit {
    @Override
    public void start(Setup setup) {

        setup.registerSubsystem(DriveSystem::new);
        setup.registerSubsystem(new Odo3Header());

        setup.registerTeleop(Odo3Header.Odo3System.class, )

        setup.registerTeleop(DriveSystem.class, OdometrySystem.class, (context, driveSystem, odometrySystem) -> new DriveDirectorTeleop(context, driveSystem, odometrySystem, visionSystem));


    }
}
