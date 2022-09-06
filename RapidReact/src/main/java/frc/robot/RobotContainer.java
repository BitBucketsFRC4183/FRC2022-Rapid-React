package frc.robot;

import frc.robot.drive.odometry.Odometry;
import frc.robot.lib.RobotInit;
import frc.robot.lib.resources.b.Exclusive;
import frc.robot.lib.resources.b.Exclusives;

public class RobotContainer implements RobotInit {

    @Override
    public void start(Setup setup) {

        setup.initReadSystem(Odometry.HEADER)


    }
}
