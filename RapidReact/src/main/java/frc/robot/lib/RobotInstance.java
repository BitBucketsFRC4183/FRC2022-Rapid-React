package frc.robot.lib;

import java.util.Optional;

public interface RobotInstance {

    Optional<Throwable> borrowCheck();
    String graph(); //TODO implement graph building from registered subsystems and commands

    void periodic();

}
